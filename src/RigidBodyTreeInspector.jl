__precompile__()

module RigidBodyTreeInspector

using RigidBodyDynamics
using RigidBodyDynamics: OdeIntegrators
import RigidBodyDynamics: parse_urdf  # deprecated import, will eventually be removed
import DrakeVisualizer: Visualizer,
                        settransform!,
                        setgeometry!,
                        addgeometry!,
                        batch,
                        GeometryData,
                        Triad,
                        HyperEllipsoid,
                        center
using StaticArrays: SVector, SMatrix
using CoordinateTransformations: AffineMap, IdentityTransformation, AngleAxis,
                                 LinearMap, RodriguesVec, Quat, compose,
                                 Translation, RotMatrix, RotXYZ, transform_deriv
using GeometryTypes: AbstractGeometry,
                     AbstractMesh,
                     HyperRectangle,
                     HyperSphere,
                     Vec, Point
using ColorTypes: RGBA
import Interact
import Interpolations: interpolate, Linear, Gridded
import Base: convert, *, one
import Rotations
import LoopThrottle: @throttle
using MechanismGeometries: VisualElement, Skeleton, AbstractGeometrySource, 
                           visual_elements, URDFVisuals

const rbd = RigidBodyDynamics

rbd.parse_urdf

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
       DrakeVisualizerSink,
       visual_elements,
       Skeleton,
       URDFVisuals

include("manipulate.jl")
include("visualizer.jl")
include("animate.jl")
include("ode_callback.jl")

end
