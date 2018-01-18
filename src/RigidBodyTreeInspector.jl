__precompile__()

module RigidBodyTreeInspector

using FileIO
using RigidBodyDynamics
import RigidBodyDynamics: parse_urdf,
                          spatial_inertia,
                          has_defined_inertia,
                          default_frame,
                          OdeIntegrators
using RigidBodyDynamics.Graphs
import DrakeVisualizer: Visualizer,
                        settransform!,
                        setgeometry!,
                        addgeometry!,
                        batch,
                        GeometryData,
                        HyperEllipsoid,
                        HyperCylinder,
                        Triad
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
import LoopThrottle: @throttle

const rbd = RigidBodyDynamics

export manipulate,
       inspect,
       inspect!,
       Visualizer,
       settransform!,
       addgeometry!,
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
