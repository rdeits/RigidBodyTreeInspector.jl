function parse_scalar{T}(::Type{T}, e::XMLElement, name::ASCIIString)
    T(parse(attribute(e, name)))
end

function parse_scalar{T}(::Type{T}, e::XMLElement, name::ASCIIString, default::ASCIIString)
    T(parse(e == nothing ? default : attribute(e, name)))
end

function parse_vector{T}(::Type{T}, e::Union{XMLElement, Void}, name::ASCIIString, default::ASCIIString)
    usedefault = e == nothing || attribute(e, name) == nothing
    [T(parse(str)) for str in split(usedefault ? default : attribute(e, name), " ")]
end

function parse_pose{T}(::Type{T}, xml_pose::Union{Void, XMLElement})
    if xml_pose == nothing
        rot = one(Quaternion{T})
        trans = zero(Vec{3, T})
    else
        rpy = parse_vector(T, xml_pose, "rpy", "0 0 0")
        rot = RigidBodyDynamics.rpy_to_quaternion(rpy)
        trans = Vec(parse_vector(T, xml_pose, "xyz", "0 0 0"))
    end
    rot, trans
end

function parse_geometry{T}(::Type{T}, xml_geometry::XMLElement)
    geometries = AbstractGeometry{3, T}[]
    for xml_cylinder in get_elements_by_tagname(xml_geometry, "cylinder")
        length = parse_scalar(Float64, xml_cylinder, "length")
        radius = parse_scalar(Float64, xml_cylinder, "radius")
        push!(geometries, HyperCylinder{3, Float64}(length, radius))
    end
    for xml_box in get_elements_by_tagname(xml_geometry, "box")
        size = Vec(parse_vector(Float64, xml_box, "size", "0 0 0"))
        push!(geometries, HyperRectangle(-size / 2, size))
    end
    for xml_sphere in get_elements_by_tagname(xml_geometry, "sphere")
        radius = parse_scalar(Float64, xml_sphere, "radius")
        push!(geometries, HyperSphere(zero(Point{3, Float64}), radius))
    end
    geometries
end

function parse_material{T}(::Type{T}, xml_material, named_colors::Dict{ASCIIString, RGBA{T}})
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

function parse_urdf(filename::ASCIIString)
    xdoc = parse_file(filename)
    xroot = root(xdoc)
    @assert name(xroot) == "robot"
    xml_links = get_elements_by_tagname(xroot, "link")
    xml_materials = get_elements_by_tagname(xroot, "material")
    named_colors = [attribute(m, "name")::ASCIIString => parse_material(Float64, m)::RGBA{Float64} for m in xml_materials]
    geometry_data = GeometryData[]
    vis_data = Link[]
    for xml_link in xml_links
        xml_visuals = get_elements_by_tagname(xml_link, "visual")
        linkname = attribute(xml_link, "name")
        geometry_data = GeometryData[]
        for xml_visual in xml_visuals
            rot, trans = parse_pose(Float64, find_element(xml_visual, "origin"))
            transform = AffineTransform(Array(rotationmatrix(rot)), Array(trans))
            geometries = parse_geometry(Float64, find_element(xml_visual, "geometry"))
            color = parse_material(Float64, find_element(xml_visual, "material"), named_colors)
            for geometry in geometries
                push!(geometry_data, GeometryData(geometry, transform, color))
            end
        end
        push!(vis_data, Link(geometry_data, linkname))
    end
    return Visualizer(vis_data)
end
