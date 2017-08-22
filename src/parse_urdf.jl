function parse_scalar(::Type{T}, e::XMLElement, name::String) where T
    T(parse(attribute(e, name)))
end

function parse_scalar(::Type{T}, e::XMLElement, name::String, default::String) where T
    T(parse(e == nothing ? default : attribute(e, name)))
end

function parse_vector(::Type{T}, e::Union{XMLElement, Void}, name::String, default::String) where T
    usedefault = e == nothing || attribute(e, name) == nothing
    [T(parse(str)) for str in split(usedefault ? default : strip(attribute(e, name)), " ")]
end

function parse_pose(::Type{T}, xml_pose::Void) where T
    rot = eye(RotMatrix{3, T})
    trans = zero(SVector{3, T})
    rot, trans
end

function parse_pose(::Type{T}, xml_pose::XMLElement) where T
    rpy = RotXYZ(parse_vector(T, xml_pose, "rpy", "0 0 0")...)
    rot = RotMatrix(rpy)
    trans = SVector{3}(parse_vector(T, xml_pose, "xyz", "0 0 0"))
    rot, trans
end

function parse_geometry(::Type{T}, xml_geometry::XMLElement, package_path, file_path="") where T
    geometries = Union{AbstractGeometry, AbstractMesh}[]
    for xml_cylinder in get_elements_by_tagname(xml_geometry, "cylinder")
        length = parse_scalar(Float64, xml_cylinder, "length")
        radius = parse_scalar(Float64, xml_cylinder, "radius")
        push!(geometries, HyperCylinder{3, Float64}(length, radius))
    end
    for xml_box in get_elements_by_tagname(xml_geometry, "box")
        size = Vec{3, Float64}(parse_vector(Float64, xml_box, "size", "0 0 0"))
        push!(geometries, HyperRectangle(-size / 2, size))
    end
    for xml_sphere in get_elements_by_tagname(xml_geometry, "sphere")
        radius = parse_scalar(Float64, xml_sphere, "radius")
        push!(geometries, HyperSphere(zero(Point{3, Float64}), radius))
    end
    for xml_mesh in get_elements_by_tagname(xml_geometry, "mesh")
        filename = attribute(xml_mesh, "filename")
        dae_pattern = r".dae$"
        replaced_extension_with_obj = false
        if ismatch(dae_pattern, filename)
            filename = replace(filename, dae_pattern, ".obj")
            replaced_extension_with_obj = true
        end
        package_pattern = r"^package://"

        if ismatch(package_pattern, filename)
            found_mesh = false
            for package_directory in package_path
                filename_in_package = joinpath(package_directory, replace(filename, package_pattern, ""))
                if ispath(filename_in_package)
                    mesh = load(filename_in_package)
                    push!(geometries, mesh)
                    found_mesh = true
                    break
                end
            end
            if !found_mesh
                warning_message = "Could not find the mesh file: $(filename). I tried substituting the following folders for the 'package://' prefix: $(package_path)."
                if replaced_extension_with_obj
                    warning_message *= " Note that I replaced the file's original extension with .obj to try to find a mesh in a format I can actually load."
                end
                warn(warning_message)
            end
        else
            filename = joinpath(file_path, filename)
            if ispath(filename)
                mesh = load(filename)
                push!(geometries, mesh)
            else
                warning_message = "Could not find the mesh file: $(filename)."
                if replaced_extension_with_obj
                    warning_message *= " Note that I replaced the file's original extension with .obj to try to find a mesh in a format I can actually load."
                end
                warn(warning_message)
            end
        end
    end
    geometries
end

function parse_material(::Type{T}, xml_material, named_colors::Dict{String, RGBA{T}}=Dict{String, RGBA{T}}()) where T
    default = "0.7 0.7 0.7 1."
    if xml_material == nothing
        color = RGBA{T}(parse_vector(T, nothing, "rgba", default)...)
    else
        xml_color = find_element(xml_material, "color")
        name = attribute(xml_material, "name")
        if xml_color != nothing || !haskey(named_colors, name) # be lenient when it comes to missing color definitions
            color = RGBA{T}(parse_vector(T, xml_color, "rgba", default)...)
        else
            color = named_colors[name]
        end
    end
    color::RGBA{T}
end

ros_package_path() = split(get(ENV, "ROS_PACKAGE_PATH", ""), ':')

function create_graph(xml_links, xml_joints)
    # create graph structure of XML elements
    graph = DirectedGraph{Vertex{XMLElement}, Edge{XMLElement}}()
    vertices = Vertex.(xml_links)
    for vertex in vertices
        add_vertex!(graph, vertex)
    end
    name_to_vertex = Dict(attribute(data(v), "name") => v for v in vertices)
    for xml_joint in xml_joints
        parent = name_to_vertex[attribute(find_element(xml_joint, "parent"), "link")]
        child = name_to_vertex[attribute(find_element(xml_joint, "child"), "link")]
        add_edge!(graph, parent, child, Edge(xml_joint))
    end
    graph
end

function parse_urdf_visuals(filename::String, mechanism::Mechanism;
                            package_path=ros_package_path(), file_path="")
    xdoc = parse_file(filename)
    xroot = root(xdoc)
    @assert name(xroot) == "robot"

    xml_links = get_elements_by_tagname(xroot, "link")
    xml_joints = get_elements_by_tagname(xroot, "joint")
    xml_materials = get_elements_by_tagname(xroot, "material")

    graph = create_graph(xml_links, xml_joints)
    roots = collect(filter(v -> isempty(in_edges(v, graph)), vertices(graph)))
    length(roots) != 1 && error("Can only handle a single root")
    tree = SpanningTree(graph, first(roots))

    named_colors = Dict(attribute(m, "name")::String => parse_material(Float64, m)::RGBA{Float64} for m in xml_materials)
    name_to_frame_and_body = Dict(rbd.name(tf.from) => (tf.from, body) for body in bodies(mechanism) for tf in rbd.frame_definitions(body))

    vis_data = Dict{CartesianFrame3D, Vector{GeometryData}}()
    for vertex in vertices(tree)
        xml_link = data(vertex)
        xml_visuals = get_elements_by_tagname(xml_link, "visual")
        linkname = attribute(xml_link, "name")
        framename = if vertex == rbd.Graphs.root(tree)
            linkname
        else
            xml_joint = data(edge_to_parent(vertex, tree))
            jointname = attribute(xml_joint, "name")
            string("after_", jointname) # TODO: create function in RBD, call it here
        end

        !haskey(name_to_frame_and_body, framename) && continue
        body_frame, body = name_to_frame_and_body[framename]

        for xml_visual in xml_visuals
            geometries = parse_geometry(Float64,
                                        find_element(xml_visual, "geometry"),
                                        package_path, file_path)
            color = parse_material(Float64,
                                   find_element(xml_visual, "material"),
                                   named_colors)
            rot, trans = parse_pose(Float64, find_element(xml_visual, "origin"))
            geom_frame = CartesianFrame3D("geometry")
            add_frame!(body, Transform3D(geom_frame, body_frame, rot, trans))
            for geometry in geometries
                vis_data[geom_frame] = [GeometryData(geometry, color)]
            end
        end

    end
    return vis_data
end

"""
    parse_urdf(filename::String, mechanism::Mechanism;
               package_path=ros_package_path())

Extract the visual elements (geometric primitives and meshes) from a URDF
specified by filename.

package_path is a vector of strings, used to supply additional search paths
for `package://` URLs inside the URDF, which is how many URDFs specify the
locations of their mesh files. By default, package_path will be set from the
value of your `ROS_PACKAGE_PATH` environment variable.
"""
parse_urdf(args...; kwargs...) = parse_urdf_visuals(args...; kwargs...)
