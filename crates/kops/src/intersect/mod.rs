//! Geometric intersection algorithms and parameter-rich result contracts.

mod candidate;
mod circle_circle;
mod circle_cone;
mod circle_cylinder;
mod circle_ellipse;
mod circle_nurbs;
mod circle_sphere;
mod circle_torus;
mod cone_cone;
mod cone_cylinder;
mod cone_nurbs_surface;
mod cone_sphere;
mod cone_torus;
mod conic;
mod curve_curve;
mod curve_surface;
mod cylinder_cylinder;
mod cylinder_nurbs_surface;
mod cylinder_sphere;
mod cylinder_torus;
mod ellipse_cone;
mod ellipse_cylinder;
mod ellipse_ellipse;
mod ellipse_nurbs;
mod ellipse_sphere;
mod ellipse_torus;
mod error;
mod geometry_class;
mod graph_surface;
mod line_circle;
mod line_cone;
mod line_cylinder;
mod line_ellipse;
mod line_line;
mod line_nurbs;
mod line_plane;
mod line_sphere;
mod line_torus;
mod numerical;
mod nurbs_cone;
mod nurbs_curve_march;
mod nurbs_cylinder;
mod nurbs_nurbs;
mod nurbs_plane;
mod nurbs_sphere;
mod nurbs_surface_march;
mod nurbs_torus;
mod parameter;
mod planar_curve_plane;
mod plane_cone;
mod plane_cylinder;
mod plane_nurbs_surface;
mod plane_plane;
mod plane_sphere;
mod plane_torus;
mod result;
mod sphere_nurbs_surface;
mod sphere_sphere;
mod sphere_torus;
mod support_curve_pair;
mod surface_surface;
mod torus_nurbs_surface;
mod torus_torus;

pub use circle_circle::intersect_bounded_circles;
pub use circle_cone::intersect_bounded_circle_cone;
pub use circle_cylinder::intersect_bounded_circle_cylinder;
pub use circle_ellipse::intersect_bounded_circle_ellipse;
pub use circle_nurbs::intersect_bounded_circle_nurbs;
pub use circle_sphere::intersect_bounded_circle_sphere;
pub use circle_torus::intersect_bounded_circle_torus;
pub use cone_cone::intersect_bounded_cones;
pub use cone_cylinder::intersect_bounded_cone_cylinder;
pub use cone_nurbs_surface::intersect_bounded_cone_nurbs_surface;
pub use cone_sphere::intersect_bounded_cone_sphere;
pub use cone_torus::intersect_bounded_cone_torus;
pub use curve_curve::{
    CurveCurveBudgetProfile, intersect_bounded_curves, intersect_bounded_curves_in_scope,
    intersect_bounded_curves_with_context,
};
pub use curve_surface::intersect_bounded_curve_surface;
pub use cylinder_cylinder::intersect_bounded_cylinders;
pub use cylinder_nurbs_surface::intersect_bounded_cylinder_nurbs_surface;
pub use cylinder_sphere::intersect_bounded_cylinder_sphere;
pub use cylinder_torus::intersect_bounded_cylinder_torus;
pub use ellipse_cone::intersect_bounded_ellipse_cone;
pub use ellipse_cylinder::intersect_bounded_ellipse_cylinder;
pub use ellipse_ellipse::{intersect_bounded_ellipses, intersect_bounded_ellipses_with_context};
pub use ellipse_nurbs::intersect_bounded_ellipse_nurbs;
pub use ellipse_sphere::intersect_bounded_ellipse_sphere;
pub use ellipse_torus::intersect_bounded_ellipse_torus;
pub use error::{
    CURVE_CURVE_CLASS_PAIR, CURVE_SURFACE_CLASS_PAIR, IntersectionError, IntersectionResult,
    SURFACE_SURFACE_CLASS_PAIR, UNSUPPORTED_CLASS_PAIR,
};
pub use graph_surface::{
    BRANCH_CERTIFICATE_FAILURE, GraphSurfaceBudgetProfile, GraphSurfaceIntersectionError,
    GraphSurfaceIntersectionResult, GraphSurfaceSurfaceIntersections,
    IntersectionBranchCertificate, IntersectionBranchEdge, IntersectionBranchEndpointEvent,
    IntersectionBranchGraph, IntersectionBranchVertex, IntersectionBranchVertexEvent,
    NURBS_TRACE_CERTIFICATE_WORK, PERSISTENT_DESCRIPTOR_FAILURE, PersistentIntersectionBranchEdge,
    PersistentIntersectionBranchGraph, SPHERICAL_CIRCLE_PROOF_SUBDIVISIONS,
    intersect_bounded_graph_surfaces, intersect_bounded_graph_surfaces_in_scope,
    intersect_bounded_graph_surfaces_with_context, persist_verified_graph_surface_intersections,
};
pub use kgraph::{CurveClass, GeometryClassKey, SurfaceClass};
pub use line_circle::intersect_bounded_line_circle;
pub use line_cone::intersect_bounded_line_cone;
pub use line_cylinder::intersect_bounded_line_cylinder;
pub use line_ellipse::intersect_bounded_line_ellipse;
pub use line_line::intersect_bounded_lines;
pub use line_nurbs::intersect_bounded_line_nurbs;
pub use line_plane::intersect_bounded_line_plane;
pub use line_sphere::intersect_bounded_line_sphere;
pub use line_torus::intersect_bounded_line_torus;
pub use nurbs_cone::intersect_bounded_nurbs_cone;
pub use nurbs_cylinder::intersect_bounded_nurbs_cylinder;
pub use nurbs_nurbs::{
    NURBS_CURVE_PAIR_COMPLETE_COVERAGE, NURBS_CURVE_PAIR_COVERAGE_INCOMPLETE,
    NURBS_CURVE_PAIR_ISOLATION_CANDIDATE_LIMIT, NURBS_CURVE_PAIR_ISOLATION_DEPTH_LIMIT,
    NURBS_CURVE_PAIR_ISOLATION_METHOD_UNAVAILABLE, NURBS_CURVE_PAIR_ISOLATION_PARAMETER_RESOLUTION,
    NURBS_CURVE_PAIR_ISOLATION_SUBDIVISION_LIMIT, NURBS_CURVE_PAIR_MINIMIZER_DIAGNOSTICS,
    NURBS_CURVE_PAIR_MINIMIZER_INVALID_OBJECTIVE, NURBS_CURVE_PAIR_MINIMIZER_ITERATION_LIMIT,
    NURBS_CURVE_PAIR_MINIMIZER_PARAMETER_RESOLUTION, NURBS_CURVE_PAIR_OVERLAP_EQUIVALENCE,
    NURBS_CURVE_PAIR_OVERLAP_EQUIVALENCE_LIMIT, NURBS_CURVE_PAIR_POLISH_DIAGNOSTICS,
    NURBS_CURVE_PAIR_POLISH_FALLBACK, NURBS_CURVE_PAIR_POLISH_ILL_CONDITIONED,
    NURBS_CURVE_PAIR_POLISH_ITERATION_LIMIT, NURBS_CURVE_PAIR_POLISH_NO_DESCENT,
    NURBS_CURVE_PAIR_POLISH_PARAMETER_RESOLUTION, NURBS_CURVE_PAIR_POLISH_STATIONARY,
    NURBS_CURVE_PAIR_PROOF_DIAGNOSTICS, NURBS_CURVE_PAIR_SEED_ATTEMPTS,
    NURBS_CURVE_PAIR_SEED_LIMIT, intersect_bounded_nurbs_nurbs,
    intersect_bounded_nurbs_nurbs_with_context,
};
pub use nurbs_plane::intersect_bounded_nurbs_plane;
pub use nurbs_sphere::intersect_bounded_nurbs_sphere;
pub use nurbs_surface_march::{
    NURBS_SURFACE_MARCH_CAPABILITIES, NURBS_SURFACE_MARCH_COMPLETE_COVERAGE,
    NURBS_SURFACE_MARCH_DIAGNOSTICS, NURBS_SURFACE_MARCH_INCOMPLETE,
    NURBS_SURFACE_MARCH_SAMPLE_LIMIT, NURBS_SURFACE_MARCH_SAMPLES, NurbsSurfaceMarchBudgetProfile,
};
pub use nurbs_torus::intersect_bounded_nurbs_torus;
pub use planar_curve_plane::{intersect_bounded_circle_plane, intersect_bounded_ellipse_plane};
pub use plane_cone::intersect_bounded_plane_cone;
pub use plane_cylinder::intersect_bounded_plane_cylinder;
pub use plane_nurbs_surface::intersect_bounded_plane_nurbs_surface;
pub use plane_nurbs_surface::intersect_bounded_plane_nurbs_surface_with_context;
pub use plane_plane::intersect_bounded_planes;
pub use plane_sphere::intersect_bounded_plane_sphere;
pub use plane_torus::intersect_bounded_plane_torus;
pub use result::{
    ContactKind, CurveCurveIntersections, CurveCurveOverlap, CurveCurvePoint,
    CurveSurfaceIntersections, CurveSurfaceOverlap, CurveSurfacePoint, OrthogonalSphereOctantMap,
    ParamOrientation, SurfaceIntersectionCurve, SurfaceRegionCorrespondence,
    SurfaceRegionOrientation, SurfaceSurfaceCurve, SurfaceSurfaceIntersections,
    SurfaceSurfacePoint, SurfaceSurfaceRegion, SurfaceSurfaceRegionVertex,
    accept_curve_curve_candidate, accept_curve_surface_candidate, accept_surface_surface_candidate,
};
pub use sphere_nurbs_surface::intersect_bounded_sphere_nurbs_surface;
pub use sphere_nurbs_surface::intersect_bounded_sphere_nurbs_surface_with_context;
pub use sphere_sphere::intersect_bounded_spheres;
pub use sphere_torus::intersect_bounded_sphere_torus;
pub use surface_surface::intersect_bounded_surfaces;
pub use torus_nurbs_surface::intersect_bounded_torus_nurbs_surface;
pub use torus_torus::intersect_bounded_tori;
