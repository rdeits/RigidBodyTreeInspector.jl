module RigidBodyTreeInspector

using FileIO
using RigidBodyDynamics
import RigidBodyDynamics: parse_urdf,
                          spatial_inertia,
                          has_defined_inertia,
                          default_frame,
                          OdeIntegrators
import DrakeVisualizer: Visualizer,
                        settransform!,
                        setgeometry!,
                        batch,
                        GeometryData,
                        HyperEllipsoid,
                        HyperCylinder
import StaticArrays: SVector, SMatrix
import CoordinateTransformations: AffineMap, IdentityTransformation, AngleAxis,
                                  LinearMap, RodriguesVec, Quat, compose,
                                  Translation, RotMatrix, RotXYZ
import GeometryTypes: AbstractGeometry,
                      AbstractMesh,
                      HyperRectangle, 
                      HyperSphere,
                      Vec, Point
import DataStructures: OrderedDict
import ColorTypes: RGBA
import Interact
import Interpolations: interpolate, Linear, Gridded
import Base: convert, *, one
import LightXML: XMLElement, parse_file, root, get_elements_by_tagname,
                 attribute, find_element, name
import Rotations
import MeshIO

export manipulate,
       inspect,
       inspect!,
       Visualizer,
       settransform!,
       setgeometry!,
       animate,
       parse_urdf,
       create_geometry,
       DrakeVisualizerSink

include("geometry.jl")
include("manipulate.jl")
include("visualizer.jl")
include("animate.jl")
include("parse_urdf.jl")
include("ode_callback.jl")

end
