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

function parse_pose{T}(::Type{T}, xmlPose::Union{Void, XMLElement})
    if xmlPose == nothing
        rot = one(Quaternion{T})
        trans = zero(Vec{3, T})
    else
        rpy = parse_vector(T, xmlPose, "rpy", "0 0 0")
        rot = rpy_to_quaternion(rpy)
        trans = Vec(parse_vector(T, xmlPose, "xyz", "0 0 0"))
    end
    rot, trans
end

function parse_geometry{T}(::Type{T}, xmlGeometry::XMLElement)
    geometries = GeometryTypes.AbstractGeometry{3, T}[]
    for xml_cylinder in get_elements_by_tagname(xmlGeometry, "cylinder")
        length = parse_scalar(Float64, xml_cylinder, "length")
        radius = parse_scalar(Float64, xml_cylinder, "radius")
        push!(geometries, HyperCylinder{3, Float64}(length, radius))
    end
    for xml_box in get_elements_by_tagname(xmlGeometry, "box")
        size = Vec(parse_vector(Float64, xml_box, "size", "0 0 0"))
        push!(geometries, GeometryTypes.HyperRectangle(-size / 2, size))
    end
    for xml_sphere in get_elements_by_tagname(xmlGeometry, "sphere")
        radius = parse_scalar(Float64, xml_sphere, "radius")
        push!(geometries, GeometryTypes.HyperSphere(zero(Point{3, Float64}), radius))
    end
    geometries
end

function parse_material{T}(::Type{T}, xmlMaterial, namedColors::Dict{ASCIIString, ColorTypes.RGBA{T}})
    default = "0.7 0.7 0.7 1."
    if xmlMaterial == nothing
        color = ColorTypes.RGBA{T}(parse_vector(T, nothing, "rgba", default)...)
    else
        xmlColor = find_element(xmlMaterial, "color")
        name = attribute(xmlMaterial, "name")
        if xmlColor != nothing || !haskey(namedColors, name) # be lenient when it comes to missing color definitions
            color = ColorTypes.RGBA{T}(parse_vector(T, xmlColor, "rgba", default)...)
        else
            color = namedColors[name]
        end
    end
    color::ColorTypes.RGBA{T}
end

function parse_urdf(filename::ASCIIString, mechanism::Mechanism)
    xdoc = parse_file(filename)
    xroot = root(xdoc)
    @assert LightXML.name(xroot) == "robot"
    xmlLinks = get_elements_by_tagname(xroot, "link")
    xmlMaterials = get_elements_by_tagname(xroot, "material")
    namedColors = [attribute(m, "name")::ASCIIString => parse_material(Float64, m)::ColorTypes.RGBA{Float64} for m in xmlMaterials]
    bods = bodies(mechanism)
    geometrydata = GeometryData[]
    visdata = Link[]
    for xmlLink in xmlLinks
        xmlVisuals = get_elements_by_tagname(xmlLink, "visual")
        linkname = attribute(xmlLink, "name")
        body = bods[findfirst(b -> RigidBodyDynamics.name(b) == linkname, bods)]
        if !isroot(body) # geometry attached to world is currently not supported. Also, this is the only reason to have the Mechanism argument for this function
            geometrydata = GeometryData[]
            for xmlVisual in xmlVisuals
                rot, trans = RigidBodyDynamics.parse_pose(Float64, find_element(xmlVisual, "origin"))
                transform = AffineTransform(Array(rotationmatrix(rot)), Array(trans))
                geometries = parse_geometry(Float64, find_element(xmlVisual, "geometry"))
                color = parse_material(Float64, find_element(xmlVisual, "material"), namedColors)
                for geometry in geometries
                    push!(geometrydata, GeometryData(geometry, transform, color))
                end
            end
            push!(visdata, Link(geometrydata, linkname))
        end
    end
    return Visualizer(visdata)
end
