# __precompile__()  # precompilation blocked by https://github.com/JuliaGeometry/Quaternions.jl/issues/18 via RigidBodyDynamics

module RigidBodyTreeInspector

using FileIO
using RigidBodyDynamics
import RigidBodyDynamics: parse_urdf, edge_to_parent_data, vertex_data
import DrakeVisualizer: Visualizer, draw, Link, GeometryData, HyperEllipsoid,
                        HyperCylinder
import StaticArrays: SVector
import CoordinateTransformations: AffineMap, IdentityTransformation, AngleAxis,
                                  LinearMap, RodriguesVec, Quat, compose,
                                  Translation
import GeometryTypes: AbstractGeometry, HyperRectangle, HyperSphere, Vec, Point
import DataStructures: OrderedDict
import ColorTypes: RGBA
import Interact
import Interpolations: interpolate, Linear, Gridded
import Base: convert, *, one
import LightXML: XMLElement, parse_file, root, get_elements_by_tagname,
                 attribute, find_element, name
import MeshIO

export manipulate,
       inspect,
       Visualizer,
       draw,
       animate,
       parse_urdf

include("geometry.jl")
include("manipulate.jl")
include("visualizer.jl")
include("animate.jl")
include("parse_urdf.jl")

end
