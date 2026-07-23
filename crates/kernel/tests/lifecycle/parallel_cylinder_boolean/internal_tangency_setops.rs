//! Facade-only set-operation evidence for exact internal radial tangency.
//!
//! `O` denotes the larger containing radial disk and `I` the smaller
//! contained disk. The supported slice keeps whole sources, canonical finite
//! `I`-radius bands, and exact tangent-shoulder chains when `I` has one or two
//! axial protrusions. The pinched `O \ I` annulus remains a typed, atomic
//! refusal.
//! Wall-time budget: less than 60 seconds for the semantic and rigid-frame
//! matrices.

use super::*;

const INTERNAL_TANGENCY_RELATION_WORK: u64 = 64;
const INTERNAL_TANGENCY_BAND_WORK: u64 = 420;
const INTERNAL_TANGENCY_SHOULDER_WORK: u64 = 1_092;
const INTERNAL_TANGENCY_CHAIN_WORK: u64 = 2_052;
const INTERNAL_TANGENCY_PROPERTIES_WORK: u64 = 3_953;
const INTERNAL_TANGENCY_SHOULDER_PROPERTIES_WORK: u64 = 7_881;
const INTERNAL_TANGENCY_CHAIN_PROPERTIES_WORK: u64 = 11_809;
const WHOLE_CYLINDER_COPY_WORK: u64 = 26;
const WHOLE_CYLINDER_COPY_IDENTITIES: usize = 26;
const CONTAINING_RADIUS: f64 = 3.0;
const CONTAINED_RADIUS: f64 = 1.0;
const CENTER_SEPARATION: f64 = 2.0;
const TANGENT_SHOULDER_TOPOLOGY: [usize; 3] = [5, 4, 1];
const TANGENT_CHAIN_TOPOLOGY: [usize; 3] = [7, 6, 2];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RadialRole {
    Containing,
    Contained,
}

impl RadialRole {
    const fn index(self) -> usize {
        match self {
            Self::Containing => 0,
            Self::Contained => 1,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum InternalRequest {
    Intersect { swapped: bool },
    Unite { swapped: bool },
    ContainedMinusContaining,
    ContainingMinusContained,
}

#[derive(Debug, Clone, Copy)]
enum ExpectedResult {
    ProvenEmpty,
    SourceCopy(RadialRole),
    RebuiltBands(&'static [[f64; 2]]),
    TangentShoulder,
    TangentChain,
    Refused,
}

#[derive(Debug, Clone, Copy)]
struct SemanticRow {
    name: &'static str,
    /// Intervals in `[O, I]` order.
    intervals: [[f64; 2]; 2],
    intersect: ExpectedResult,
    unite: ExpectedResult,
    contained_minus_containing: ExpectedResult,
}

const CROSSING: SemanticRow = SemanticRow {
    name: "crossing",
    intervals: [[-2.0, 1.0], [-1.0, 2.0]],
    intersect: ExpectedResult::RebuiltBands(&[[-1.0, 1.0]]),
    unite: ExpectedResult::TangentShoulder,
    contained_minus_containing: ExpectedResult::RebuiltBands(&[[1.0, 2.0]]),
};

const REVERSE_CROSSING: SemanticRow = SemanticRow {
    name: "reverse crossing",
    intervals: [[-1.0, 2.0], [-2.0, 1.0]],
    intersect: ExpectedResult::RebuiltBands(&[[-1.0, 1.0]]),
    unite: ExpectedResult::TangentShoulder,
    contained_minus_containing: ExpectedResult::RebuiltBands(&[[-2.0, -1.0]]),
};

const CONTAINING_COVERS_CONTAINED: SemanticRow = SemanticRow {
    name: "O contains I axially",
    intervals: [[-2.0, 2.0], [-1.0, 1.0]],
    intersect: ExpectedResult::SourceCopy(RadialRole::Contained),
    unite: ExpectedResult::SourceCopy(RadialRole::Containing),
    contained_minus_containing: ExpectedResult::ProvenEmpty,
};

const CONTAINED_COVERS_CONTAINING: SemanticRow = SemanticRow {
    name: "I contains O axially",
    intervals: [[-1.0, 1.0], [-2.0, 2.0]],
    intersect: ExpectedResult::RebuiltBands(&[[-1.0, 1.0]]),
    unite: ExpectedResult::TangentChain,
    contained_minus_containing: ExpectedResult::RebuiltBands(&[[-2.0, -1.0], [1.0, 2.0]]),
};

const SEMANTIC_ROWS: [SemanticRow; 7] = [
    CROSSING,
    REVERSE_CROSSING,
    CONTAINING_COVERS_CONTAINED,
    CONTAINED_COVERS_CONTAINING,
    SemanticRow {
        name: "shared low with I extending high",
        intervals: [[-2.0, 0.0], [-2.0, 2.0]],
        intersect: ExpectedResult::RebuiltBands(&[[-2.0, 0.0]]),
        unite: ExpectedResult::TangentShoulder,
        contained_minus_containing: ExpectedResult::RebuiltBands(&[[0.0, 2.0]]),
    },
    SemanticRow {
        name: "shared high with I extending low",
        intervals: [[0.0, 2.0], [-2.0, 2.0]],
        intersect: ExpectedResult::RebuiltBands(&[[0.0, 2.0]]),
        unite: ExpectedResult::TangentShoulder,
        contained_minus_containing: ExpectedResult::RebuiltBands(&[[-2.0, 0.0]]),
    },
    SemanticRow {
        name: "equal axial intervals",
        intervals: [[-1.0, 1.0], [-1.0, 1.0]],
        intersect: ExpectedResult::SourceCopy(RadialRole::Contained),
        unite: ExpectedResult::SourceCopy(RadialRole::Containing),
        contained_minus_containing: ExpectedResult::ProvenEmpty,
    },
];

fn internal_tangency_frame(placement: Placement) -> Frame {
    match placement {
        Placement::World => Frame::world(),
        Placement::Oblique => Frame::new(
            Point3::new(0.5, 0.0, 0.0),
            Vec3::new(0.0, 0.28, 0.96),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
    }
}

fn operand_for_role(containing_operand: usize, role: RadialRole) -> usize {
    match role {
        RadialRole::Containing => containing_operand,
        RadialRole::Contained => 1 - containing_operand,
    }
}

fn operand_body(fixture: &Fixture, operand: usize) -> BodyId {
    match operand {
        0 => fixture.outer.clone(),
        1 => fixture.inner.clone(),
        _ => panic!("parallel-cylinder fixture has exactly two operands"),
    }
}

fn role_body(fixture: &Fixture, containing_operand: usize, role: RadialRole) -> BodyId {
    operand_body(fixture, operand_for_role(containing_operand, role))
}

fn internal_tangency_fixture(
    placement: Placement,
    intervals: [[f64; 2]; 2],
    containing_operand: usize,
    reversed_axes: [bool; 2],
) -> Fixture {
    internal_tangency_fixture_with_radial_geometry(
        placement,
        intervals,
        containing_operand,
        reversed_axes,
        [CONTAINING_RADIUS, CONTAINED_RADIUS],
        CENTER_SEPARATION,
    )
}

fn internal_tangency_fixture_with_radial_geometry(
    placement: Placement,
    intervals: [[f64; 2]; 2],
    containing_operand: usize,
    reversed_axes: [bool; 2],
    radii: [f64; 2],
    center_separation: f64,
) -> Fixture {
    assert!(containing_operand < 2);
    assert!(
        radii
            .iter()
            .all(|radius| radius.is_finite() && *radius > 0.0)
    );
    assert!(center_separation.is_finite() && center_separation > 0.0);
    let frame = internal_tangency_frame(placement);
    let mut session = Kernel::new().create_session();
    let part_id = session.create_part();
    let bodies: [BodyId; 2] = {
        let mut edit = session.edit_part(part_id.clone()).unwrap();
        core::array::from_fn(|operand| {
            let role = if operand == containing_operand {
                RadialRole::Containing
            } else {
                RadialRole::Contained
            };
            let [low, high] = intervals[role.index()];
            assert!(low < high);
            let radial = match role {
                RadialRole::Containing => 0.0,
                RadialRole::Contained => center_separation,
            };
            let radius = radii[role.index()];
            let reversed = reversed_axes[operand];
            let origin = frame.point_at(radial, 0.0, if reversed { high } else { low });
            let source_frame = if reversed {
                Frame::new(origin, -frame.z(), frame.x()).unwrap()
            } else {
                Frame::new(origin, frame.z(), frame.x()).unwrap()
            };
            edit.create_cylinder(CylinderRequest::new(source_frame, radius, high - low))
                .unwrap()
                .into_result()
                .unwrap()
                .body()
        })
    };
    Fixture {
        session,
        part_id,
        outer: bodies[0].clone(),
        inner: bodies[1].clone(),
        frame,
    }
}

fn run_internal_tangency(
    fixture: &mut Fixture,
    containing_operand: usize,
    request: InternalRequest,
) -> OperationOutcome<BooleanOutcome> {
    run_internal_tangency_with_settings(
        fixture,
        containing_operand,
        request,
        OperationSettings::new(),
    )
}

fn run_internal_tangency_with_settings(
    fixture: &mut Fixture,
    containing_operand: usize,
    request: InternalRequest,
    settings: OperationSettings,
) -> OperationOutcome<BooleanOutcome> {
    let containing = role_body(fixture, containing_operand, RadialRole::Containing);
    let contained = role_body(fixture, containing_operand, RadialRole::Contained);
    let (operation, bodies) = match request {
        InternalRequest::Intersect { swapped } => (
            BooleanOperation::Intersect,
            if swapped {
                [contained, containing]
            } else {
                [containing, contained]
            },
        ),
        InternalRequest::Unite { swapped } => (
            BooleanOperation::Unite,
            if swapped {
                [contained, containing]
            } else {
                [containing, contained]
            },
        ),
        InternalRequest::ContainedMinusContaining => {
            (BooleanOperation::Subtract, [contained, containing])
        }
        InternalRequest::ContainingMinusContained => {
            (BooleanOperation::Subtract, [containing, contained])
        }
    };
    fixture
        .session
        .edit_part(fixture.part_id.clone())
        .unwrap()
        .boolean_bodies(
            BooleanBodiesRequest::new(operation, bodies[0].clone(), bodies[1].clone())
                .with_settings(settings),
        )
        .unwrap()
}

fn internal_usage_at(
    outcome: &OperationOutcome<BooleanOutcome>,
    stage: kernel::StageId,
    resource: ResourceKind,
) -> Option<u64> {
    outcome
        .report()
        .usage()
        .iter()
        .find(|usage| usage.stage == stage && usage.resource == resource)
        .map(|usage| usage.consumed)
}

#[derive(Debug, Clone)]
struct CylinderEntityLayout {
    side: JournalEntity,
    caps: [JournalEntity; 2],
    rings: [JournalEntity; 2],
    faces: Vec<JournalEntity>,
    edges: Vec<JournalEntity>,
}

fn cylinder_entity_layout(
    fixture: &Fixture,
    body: BodyId,
    reversed_axis: bool,
) -> CylinderEntityLayout {
    let part = fixture.session.part(fixture.part_id.clone()).unwrap();
    let body_view = part.body(body.clone()).unwrap();
    let faces = body_view.faces().unwrap().collect::<Vec<_>>();
    let edges = body_view.edges().unwrap().collect::<Vec<_>>();
    assert_eq!(
        [
            faces.len(),
            edges.len(),
            body_view.vertices().unwrap().len()
        ],
        [3, 2, 0]
    );

    let side = faces
        .iter()
        .find(|face| {
            let face = part.face((*face).clone()).unwrap();
            part.surface(face.surface()).unwrap().class_key().as_str()
                == "kernel.surface.cylinder.v1"
        })
        .cloned()
        .expect("canonical cylinder has one cylindrical side");
    let mut physical_rings = [edges[0].clone(), edges[1].clone()];
    if reversed_axis {
        physical_rings.swap(0, 1);
    }
    let physical_caps = physical_rings.clone().map(|ring| {
        part.edge(ring)
            .unwrap()
            .fins()
            .map(|fin| {
                let loop_id = part.fin(fin).unwrap().loop_();
                part.loop_(loop_id).unwrap().face()
            })
            .find(|face| *face != side)
            .expect("ring edge has one planar cap use")
    });
    CylinderEntityLayout {
        side: JournalEntity::Face(side),
        caps: physical_caps.map(JournalEntity::Face),
        rings: physical_rings.map(JournalEntity::Edge),
        faces: faces.into_iter().map(JournalEntity::Face).collect(),
        edges: edges.into_iter().map(JournalEntity::Edge).collect(),
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
enum OwnedLineage {
    Derived {
        derived: JournalEntity,
        source: JournalEntity,
    },
    Split {
        source: JournalEntity,
        pieces: Vec<JournalEntity>,
    },
    Merge {
        sources: Vec<JournalEntity>,
        result: JournalEntity,
    },
}

fn owned_lineage(created: &kernel::BooleanCreatedResult) -> Vec<OwnedLineage> {
    created
        .journal()
        .lineage()
        .map(|event| match event {
            LineageView::DerivedFrom { derived, source } => {
                OwnedLineage::Derived { derived, source }
            }
            LineageView::Split { source, pieces } => OwnedLineage::Split {
                source,
                pieces: pieces.collect(),
            },
            LineageView::Merge { sources, result } => OwnedLineage::Merge {
                sources: sources.collect(),
                result,
            },
            other => panic!("internal-tangency result published unexpected lineage {other:?}"),
        })
        .collect()
}

fn assert_whole_source_copy_lineage(
    fixture: &Fixture,
    created: &kernel::BooleanCreatedResult,
    expected_source: BodyId,
) {
    assert_eq!(created.bodies().len(), 1);
    let mutations = created.journal().mutations().collect::<Vec<_>>();
    assert_eq!(mutations.len(), WHOLE_CYLINDER_COPY_IDENTITIES);
    assert!(
        mutations
            .iter()
            .all(|mutation| mutation.kind() == MutationKind::Created)
    );
    let events = owned_lineage(created);
    assert_eq!(events.len(), mutations.len());
    let mut derived = Vec::with_capacity(events.len());
    let mut body_source = None;
    let source_layout = cylinder_entity_layout(fixture, expected_source.clone(), false);
    for event in events {
        let OwnedLineage::Derived {
            derived: result,
            source,
        } = event
        else {
            panic!("whole-cylinder copy must publish only DerivedFrom")
        };
        assert_eq!(result.kind(), source.kind());
        assert!(!derived.contains(&result));
        match (&result, &source) {
            (JournalEntity::Body(body), JournalEntity::Body(source)) => {
                assert_eq!(body, &created.bodies()[0]);
                body_source = Some(source.clone());
            }
            (JournalEntity::Face(_), _) => assert!(source_layout.faces.contains(&source)),
            (JournalEntity::Edge(_), _) => assert!(source_layout.edges.contains(&source)),
            _ => {}
        }
        derived.push(result);
    }
    assert_eq!(body_source, Some(expected_source));
    assert!(
        mutations
            .iter()
            .all(|mutation| derived.contains(mutation.entity()))
    );
}

fn assert_derived(
    events: &[OwnedLineage],
    result: &JournalEntity,
    source: &JournalEntity,
    label: &str,
) {
    assert_eq!(
        events
            .iter()
            .filter(|event| matches!(
                event,
                OwnedLineage::Derived { derived, source: actual }
                    if derived == result && actual == source
            ))
            .count(),
        1,
        "{label}: missing unique DerivedFrom({result:?}, {source:?})"
    );
}

fn endpoint_sources(
    parameter: f64,
    intervals: [[f64; 2]; 2],
    containing_operand: usize,
    source_layouts: &[CylinderEntityLayout; 2],
) -> (JournalEntity, Vec<JournalEntity>) {
    let contained_operand = operand_for_role(containing_operand, RadialRole::Contained);
    for endpoint in 0..2 {
        if intervals[RadialRole::Contained.index()][endpoint].to_bits() == parameter.to_bits() {
            return (
                source_layouts[contained_operand].caps[endpoint].clone(),
                vec![source_layouts[contained_operand].rings[endpoint].clone()],
            );
        }
    }
    let containing_operand = operand_for_role(containing_operand, RadialRole::Containing);
    for endpoint in 0..2 {
        if intervals[RadialRole::Containing.index()][endpoint].to_bits() == parameter.to_bits() {
            let cap = source_layouts[containing_operand].caps[endpoint].clone();
            return (
                cap.clone(),
                vec![source_layouts[contained_operand].side.clone(), cap],
            );
        }
    }
    panic!("selected endpoint must be topology-authored")
}

fn assert_rebuilt_band_lineage(
    fixture: &Fixture,
    created: &kernel::BooleanCreatedResult,
    intervals: [[f64; 2]; 2],
    containing_operand: usize,
    reversed_axes: [bool; 2],
    spans: &[[f64; 2]],
    label: &str,
) {
    let source_layouts: [CylinderEntityLayout; 2] = core::array::from_fn(|operand| {
        cylinder_entity_layout(
            fixture,
            operand_body(fixture, operand),
            reversed_axes[operand],
        )
    });
    let result_layouts = created
        .bodies()
        .iter()
        .cloned()
        .map(|body| cylinder_entity_layout(fixture, body, false))
        .collect::<Vec<_>>();
    assert_eq!(result_layouts.len(), spans.len(), "{label}");
    let events = owned_lineage(created);
    let result_entities = result_layouts
        .iter()
        .flat_map(|layout| layout.faces.iter().chain(&layout.edges))
        .cloned()
        .collect::<Vec<_>>();
    let source_entities = source_layouts
        .iter()
        .flat_map(|layout| layout.faces.iter().chain(&layout.edges))
        .cloned()
        .collect::<Vec<_>>();
    for event in &events {
        match event {
            OwnedLineage::Derived { derived, source } => {
                assert!(result_entities.contains(derived), "{label}");
                assert!(source_entities.contains(source), "{label}");
            }
            OwnedLineage::Split { source, pieces } => {
                assert!(source_entities.contains(source), "{label}");
                assert_eq!(source.kind(), EntityKind::Face, "{label}");
                assert!(pieces.iter().all(|piece| result_entities.contains(piece)));
            }
            OwnedLineage::Merge { .. } => {
                panic!("{label}: unequal radial supports may not publish Merge lineage")
            }
        }
    }

    let contained_operand = operand_for_role(containing_operand, RadialRole::Contained);
    let contained_side = &source_layouts[contained_operand].side;
    let result_sides = result_layouts
        .iter()
        .map(|layout| layout.side.clone())
        .collect::<Vec<_>>();
    let mut expected_event_count = 0;
    if spans.len() == 2 {
        assert_eq!(
            events
                .iter()
                .filter(|event| matches!(
                    event,
                    OwnedLineage::Split { source, pieces }
                        if source == contained_side && pieces == &result_sides
                ))
                .count(),
            1,
            "{label}: two bands must be the ordered Split pieces of I's side"
        );
        expected_event_count += 1;
    } else {
        assert_eq!(spans.len(), 1, "{label}");
        assert_derived(&events, &result_sides[0], contained_side, label);
        expected_event_count += 1;
    }

    for (layout, span) in result_layouts.iter().zip(spans) {
        for endpoint in 0..2 {
            let (cap_source, ring_sources) = endpoint_sources(
                span[endpoint],
                intervals,
                containing_operand,
                &source_layouts,
            );
            assert_derived(&events, &layout.caps[endpoint], &cap_source, label);
            expected_event_count += 1;
            let actual_ring_sources = events
                .iter()
                .filter_map(|event| match event {
                    OwnedLineage::Derived { derived, source }
                        if derived == &layout.rings[endpoint] =>
                    {
                        Some(source.clone())
                    }
                    _ => None,
                })
                .collect::<Vec<_>>();
            assert_eq!(
                actual_ring_sources, ring_sources,
                "{label}: ring dependencies changed"
            );
            expected_event_count += actual_ring_sources.len();
        }
    }
    assert_eq!(
        events.len(),
        expected_event_count,
        "{label}: rebuilt-band lineage contained an unexpected event"
    );
}

fn derived_results(
    events: &[OwnedLineage],
    source: &JournalEntity,
    kind: EntityKind,
) -> Vec<JournalEntity> {
    events
        .iter()
        .filter_map(|event| match event {
            OwnedLineage::Derived {
                derived,
                source: actual,
            } if actual == source && derived.kind() == kind => Some(derived.clone()),
            _ => None,
        })
        .collect()
}

fn assert_tangent_shoulder_lineage(
    fixture: &Fixture,
    created: &kernel::BooleanCreatedResult,
    intervals: [[f64; 2]; 2],
    containing_operand: usize,
    reversed_axes: [bool; 2],
    label: &str,
) {
    let source_layouts: [CylinderEntityLayout; 2] = core::array::from_fn(|operand| {
        cylinder_entity_layout(
            fixture,
            operand_body(fixture, operand),
            reversed_axes[operand],
        )
    });
    let outer_operand = operand_for_role(containing_operand, RadialRole::Containing);
    let inner_operand = operand_for_role(containing_operand, RadialRole::Contained);
    let outer = &source_layouts[outer_operand];
    let inner = &source_layouts[inner_operand];
    let high_tail =
        intervals[RadialRole::Contained.index()][1] > intervals[RadialRole::Containing.index()][1];
    let contact = usize::from(high_tail);
    let far = 1 - contact;

    let events = owned_lineage(created);
    assert_eq!(events.len(), 10, "{label}: shoulder lineage changed");
    assert!(
        events
            .iter()
            .all(|event| matches!(event, OwnedLineage::Derived { .. })),
        "{label}: unequal tangent supports may publish only DerivedFrom lineage"
    );

    for source in [
        &outer.side,
        &inner.side,
        &outer.caps[contact],
        &outer.caps[far],
        &inner.caps[contact],
    ] {
        assert_eq!(
            derived_results(&events, source, EntityKind::Face).len(),
            1,
            "{label}: expected one result face from {source:?}"
        );
    }
    for source in [
        &outer.rings[contact],
        &outer.rings[far],
        &inner.rings[contact],
    ] {
        assert_eq!(
            derived_results(&events, source, EntityKind::Edge).len(),
            1,
            "{label}: expected one result edge from {source:?}"
        );
    }
    let inner_cut_from_side = derived_results(&events, &inner.side, EntityKind::Edge);
    let inner_cut_from_cap = derived_results(&events, &outer.caps[contact], EntityKind::Edge);
    assert_eq!(inner_cut_from_side.len(), 1, "{label}");
    assert_eq!(inner_cut_from_side, inner_cut_from_cap, "{label}");

    let part = fixture.session.part(fixture.part_id.clone()).unwrap();
    let body = part.body(created.bodies()[0].clone()).unwrap();
    let results = body
        .faces()
        .unwrap()
        .map(JournalEntity::Face)
        .chain(body.edges().unwrap().map(JournalEntity::Edge))
        .collect::<Vec<_>>();
    assert_eq!(results.len(), 9, "{label}");
    assert!(
        results
            .iter()
            .all(|result| events.iter().any(|event| matches!(
                event,
                OwnedLineage::Derived { derived, .. } if derived == result
            ))),
        "{label}: every result face and edge needs source lineage"
    );
}

fn assert_tangent_chain_lineage(
    fixture: &Fixture,
    created: &kernel::BooleanCreatedResult,
    containing_operand: usize,
    reversed_axes: [bool; 2],
    label: &str,
) {
    let source_layouts: [CylinderEntityLayout; 2] = core::array::from_fn(|operand| {
        cylinder_entity_layout(
            fixture,
            operand_body(fixture, operand),
            reversed_axes[operand],
        )
    });
    let outer = &source_layouts[operand_for_role(containing_operand, RadialRole::Containing)];
    let inner = &source_layouts[operand_for_role(containing_operand, RadialRole::Contained)];
    let events = owned_lineage(created);

    assert_eq!(events.len(), 14, "{label}: two-shoulder lineage changed");
    assert!(
        events
            .iter()
            .all(|event| !matches!(event, OwnedLineage::Merge { .. })),
        "{label}: tangent chain must not merge source entities"
    );

    let inner_tail_sides = events
        .iter()
        .filter_map(|event| match event {
            OwnedLineage::Split { source, pieces } if source == &inner.side => Some(pieces.clone()),
            _ => None,
        })
        .collect::<Vec<_>>();
    let [inner_tail_sides] = inner_tail_sides.as_slice() else {
        panic!("{label}: I's side must publish one ordered two-piece Split")
    };
    assert_eq!(inner_tail_sides.len(), 2, "{label}");
    assert!(
        inner_tail_sides
            .iter()
            .all(|piece| piece.kind() == EntityKind::Face),
        "{label}: I-side Split pieces must be faces"
    );

    assert_eq!(
        derived_results(&events, &outer.side, EntityKind::Face).len(),
        1,
        "{label}: O's central side lineage changed"
    );
    for endpoint in 0..2 {
        assert_eq!(
            derived_results(&events, &outer.caps[endpoint], EntityKind::Face).len(),
            1,
            "{label}: shoulder face {endpoint} must derive from O's cap"
        );
        assert_eq!(
            derived_results(&events, &inner.caps[endpoint], EntityKind::Face).len(),
            1,
            "{label}: far I cap {endpoint} lineage changed"
        );
        assert_eq!(
            derived_results(&events, &outer.rings[endpoint], EntityKind::Edge).len(),
            1,
            "{label}: O contact ring {endpoint} lineage changed"
        );
        assert_eq!(
            derived_results(&events, &inner.rings[endpoint], EntityKind::Edge).len(),
            1,
            "{label}: far I ring {endpoint} lineage changed"
        );
        assert_eq!(
            derived_results(&events, &outer.caps[endpoint], EntityKind::Edge).len(),
            1,
            "{label}: generated I contact ring {endpoint} needs O-cap lineage"
        );
    }

    let inner_contact_edges = derived_results(&events, &inner.side, EntityKind::Edge);
    let cap_contact_edges = outer
        .caps
        .iter()
        .flat_map(|cap| derived_results(&events, cap, EntityKind::Edge))
        .collect::<Vec<_>>();
    assert_eq!(inner_contact_edges.len(), 2, "{label}");
    assert_eq!(cap_contact_edges.len(), 2, "{label}");
    assert!(
        inner_contact_edges
            .iter()
            .all(|edge| cap_contact_edges.contains(edge))
            && cap_contact_edges
                .iter()
                .all(|edge| inner_contact_edges.contains(edge)),
        "{label}: each generated I contact ring needs dual I-side/O-cap lineage"
    );

    let part = fixture.session.part(fixture.part_id.clone()).unwrap();
    let body = part.body(created.bodies()[0].clone()).unwrap();
    let results = body
        .faces()
        .unwrap()
        .map(JournalEntity::Face)
        .chain(body.edges().unwrap().map(JournalEntity::Edge))
        .collect::<Vec<_>>();
    assert_eq!(results.len(), 13, "{label}");
    assert!(
        results
            .iter()
            .all(|result| events.iter().any(|event| match event {
                OwnedLineage::Derived { derived, .. } => derived == result,
                OwnedLineage::Split { pieces, .. } => pieces.contains(result),
                OwnedLineage::Merge { .. } => false,
            })),
        "{label}: every result face and edge needs source lineage"
    );
}

fn assert_full_tangent_union(
    fixture: &Fixture,
    created: &kernel::BooleanCreatedResult,
    expected_topology: [usize; 3],
    label: &str,
) {
    assert_eq!(created.bodies().len(), 1, "{label}");
    assert_eq!(created.reports().len(), 1, "{label}");
    let committed = &created.reports()[0];
    assert_eq!(committed.body(), created.bodies()[0], "{label}");
    assert_eq!(committed.report().level(), CheckLevel::Full, "{label}");
    assert_eq!(committed.report().outcome(), CheckOutcome::Valid, "{label}");
    assert!(committed.report().faults().is_empty(), "{label}");
    assert!(committed.report().gaps().is_empty(), "{label}");
    let part = fixture.session.part(fixture.part_id.clone()).unwrap();
    assert_eq!(
        body_topology(&part, created.bodies()[0].clone()),
        expected_topology,
        "{label}"
    );
    let checked = part
        .check_body(CheckBodyRequest::new(
            created.bodies()[0].clone(),
            CheckLevel::Full,
        ))
        .unwrap()
        .into_result()
        .unwrap();
    assert_eq!(
        checked.outcome(),
        CheckOutcome::Valid,
        "{label}: {checked:#?}"
    );
}

fn assert_full_tangent_shoulder(
    fixture: &Fixture,
    created: &kernel::BooleanCreatedResult,
    label: &str,
) {
    assert_full_tangent_union(fixture, created, TANGENT_SHOULDER_TOPOLOGY, label);
}

fn assert_full_tangent_chain(
    fixture: &Fixture,
    created: &kernel::BooleanCreatedResult,
    label: &str,
) {
    assert_full_tangent_union(fixture, created, TANGENT_CHAIN_TOPOLOGY, label);
}

fn assert_full_cylinder_components(
    fixture: &Fixture,
    created: &kernel::BooleanCreatedResult,
    label: &str,
) {
    assert_eq!(created.reports().len(), created.bodies().len(), "{label}");
    let part = fixture.session.part(fixture.part_id.clone()).unwrap();
    for (body, committed) in created.bodies().iter().zip(created.reports()) {
        assert_eq!(committed.body(), *body, "{label}");
        assert_eq!(committed.report().level(), CheckLevel::Full, "{label}");
        assert_eq!(committed.report().outcome(), CheckOutcome::Valid, "{label}");
        assert!(committed.report().faults().is_empty(), "{label}");
        assert!(committed.report().gaps().is_empty(), "{label}");
        assert_eq!(
            body_topology(&part, body.clone()),
            CYLINDER_TOPOLOGY,
            "{label}"
        );
        let checked = part
            .check_body(CheckBodyRequest::new(body.clone(), CheckLevel::Full))
            .unwrap()
            .into_result()
            .unwrap();
        assert_eq!(
            checked.outcome(),
            CheckOutcome::Valid,
            "{label}: {checked:#?}"
        );
    }
}

fn radial_axis_point(frame: Frame, role: RadialRole, axial: f64) -> Point3 {
    frame.point_at(
        if role == RadialRole::Containing {
            0.0
        } else {
            CENTER_SEPARATION
        },
        0.0,
        axial,
    )
}

fn assert_internal_span_properties(fixture: &Fixture, body: BodyId, span: [f64; 2]) {
    let part = fixture.session.part(fixture.part_id.clone()).unwrap();
    let properties = certified_properties_at_exact_budget(
        &part,
        body,
        INTERNAL_TANGENCY_PROPERTIES_WORK,
        "internal-tangency band",
    );
    let radius = CONTAINED_RADIUS;
    let height = span[1] - span[0];
    let volume = core::f64::consts::PI * radius.powi(2) * height;
    assert_scalar_matches_analytic(properties.volume(), volume, "internal-tangency band volume");
    assert_scalar_matches_analytic(
        properties.surface_area(),
        2.0 * core::f64::consts::PI * radius * (height + radius),
        "internal-tangency band surface area",
    );
    assert_point_matches_analytic(
        properties.centroid(),
        radial_axis_point(
            fixture.frame,
            RadialRole::Contained,
            (span[0] + span[1]) / 2.0,
        ),
    );
    let transverse = volume * (3.0 * radius.powi(2) + height.powi(2)) / 12.0;
    let axial = volume * radius.powi(2) / 2.0;
    assert_inertia_matches_analytic(
        properties.centroidal_inertia().value(),
        properties.centroidal_inertia().error_bound(),
        rotate_tensor(
            fixture.frame,
            [
                [transverse, 0.0, 0.0],
                [0.0, transverse, 0.0],
                [0.0, 0.0, axial],
            ],
        ),
    );
}

fn assert_tangent_union_properties(
    fixture: &Fixture,
    body: BodyId,
    intervals: [[f64; 2]; 2],
    expected_work: u64,
    label: &str,
) {
    let part = fixture.session.part(fixture.part_id.clone()).unwrap();
    let properties = certified_properties_at_exact_budget(&part, body, expected_work, label);
    let outer = intervals[RadialRole::Containing.index()];
    let inner = intervals[RadialRole::Contained.index()];
    let outer_height = outer[1] - outer[0];
    let mut components = vec![(
        CONTAINING_RADIUS,
        [0.0, 0.0, (outer[0] + outer[1]) / 2.0],
        outer_height,
    )];
    if inner[0] < outer[0] {
        components.push((
            CONTAINED_RADIUS,
            [CENTER_SEPARATION, 0.0, (inner[0] + outer[0]) / 2.0],
            outer[0] - inner[0],
        ));
    }
    if inner[1] > outer[1] {
        components.push((
            CONTAINED_RADIUS,
            [CENTER_SEPARATION, 0.0, (outer[1] + inner[1]) / 2.0],
            inner[1] - outer[1],
        ));
    }
    assert!(components.len() == 2 || components.len() == 3, "{label}");
    let components = components
        .into_iter()
        .map(|(radius, center, height)| {
            let mass = core::f64::consts::PI * radius.powi(2) * height;
            (mass, radius, center, height)
        })
        .collect::<Vec<_>>();
    let volume = components.iter().map(|(mass, ..)| mass).sum::<f64>();
    let inner_height = components
        .iter()
        .skip(1)
        .map(|(_, _, _, height)| height)
        .sum::<f64>();
    let area = 2.0
        * core::f64::consts::PI
        * (CONTAINING_RADIUS * outer_height
            + CONTAINED_RADIUS * inner_height
            + CONTAINING_RADIUS.powi(2));
    assert_scalar_matches_analytic(properties.volume(), volume, &format!("{label} volume"));
    assert_scalar_matches_analytic(
        properties.surface_area(),
        area,
        &format!("{label} surface area"),
    );

    let centroid: [f64; 3] = core::array::from_fn(|axis| {
        components
            .iter()
            .map(|(mass, _, center, _)| mass * center[axis])
            .sum::<f64>()
            / volume
    });
    assert_point_matches_analytic(
        properties.centroid(),
        fixture
            .frame
            .point_at(centroid[0], centroid[1], centroid[2]),
    );

    let cylinder_inertia = |mass: f64, radius: f64, height: f64| {
        let transverse = mass * (3.0 * radius.powi(2) + height.powi(2)) / 12.0;
        let axial = mass * radius.powi(2) / 2.0;
        [
            [transverse, 0.0, 0.0],
            [0.0, transverse, 0.0],
            [0.0, 0.0, axial],
        ]
    };
    let parallel_axis = |mass: f64, center: [f64; 3]| {
        let displacement = core::array::from_fn::<_, 3, _>(|axis| center[axis] - centroid[axis]);
        let squared = displacement.iter().map(|value| value * value).sum::<f64>();
        core::array::from_fn::<_, 3, _>(|row| {
            core::array::from_fn::<_, 3, _>(|column| {
                mass * (if row == column { squared } else { 0.0 }
                    - displacement[row] * displacement[column])
            })
        })
    };
    let local = core::array::from_fn(|row| {
        core::array::from_fn(|column| {
            components
                .iter()
                .map(|(mass, radius, center, height)| {
                    cylinder_inertia(*mass, *radius, *height)[row][column]
                        + parallel_axis(*mass, *center)[row][column]
                })
                .sum()
        })
    });
    assert_inertia_matches_analytic(
        properties.centroidal_inertia().value(),
        properties.centroidal_inertia().error_bound(),
        rotate_tensor(fixture.frame, local),
    );
}

fn assert_component_order_and_interiors(
    fixture: &Fixture,
    bodies: &[BodyId],
    role: RadialRole,
    spans: &[[f64; 2]],
    label: &str,
) {
    assert_eq!(bodies.len(), spans.len(), "{label}");
    let part = fixture.session.part(fixture.part_id.clone()).unwrap();
    let midpoints = spans
        .iter()
        .map(|span| (span[0] + span[1]) / 2.0)
        .collect::<Vec<_>>();
    for (index, body) in bodies.iter().enumerate() {
        let point = radial_axis_point(fixture.frame, role, midpoints[index]);
        let classified = part
            .classify_point_in_body(ClassifyPointInBodyRequest::new(body.clone(), point))
            .unwrap()
            .into_result()
            .unwrap();
        assert_eq!(
            classified.verdict(),
            &kernel::PointBodyVerdict::Interior,
            "{label}: component {index} does not contain its low-to-high span midpoint"
        );
        for endpoint in spans[index] {
            let point = radial_axis_point(fixture.frame, role, endpoint);
            let classified = part
                .classify_point_in_body(ClassifyPointInBodyRequest::new(body.clone(), point))
                .unwrap()
                .into_result()
                .unwrap();
            assert!(
                matches!(
                    classified.verdict(),
                    kernel::PointBodyVerdict::Boundary { .. }
                ),
                "{label}: component {index} expected cap-center boundary at axial {endpoint}, got {:?}",
                classified.verdict(),
            );
        }
        let outside_offset = (spans[index][1] - spans[index][0]) / 4.0;
        for axial in [
            spans[index][0] - outside_offset,
            spans[index][1] + outside_offset,
        ] {
            let point = radial_axis_point(fixture.frame, role, axial);
            let classified = part
                .classify_point_in_body(ClassifyPointInBodyRequest::new(body.clone(), point))
                .unwrap()
                .into_result()
                .unwrap();
            assert_eq!(
                classified.verdict(),
                &kernel::PointBodyVerdict::Exterior,
                "{label}: component {index} extends beyond its expected axial span at {axial}",
            );
        }
        for (other, midpoint) in midpoints.iter().enumerate() {
            if other == index {
                continue;
            }
            let point = radial_axis_point(fixture.frame, role, *midpoint);
            let classified = part
                .classify_point_in_body(ClassifyPointInBodyRequest::new(body.clone(), point))
                .unwrap()
                .into_result()
                .unwrap();
            assert_eq!(
                classified.verdict(),
                &kernel::PointBodyVerdict::Exterior,
                "{label}: component ordering/partition changed"
            );
        }
    }
}

fn assert_tangent_union_interiors(
    fixture: &Fixture,
    body: BodyId,
    intervals: [[f64; 2]; 2],
    label: &str,
) {
    let part = fixture.session.part(fixture.part_id.clone()).unwrap();
    let outer = intervals[RadialRole::Containing.index()];
    let inner = intervals[RadialRole::Contained.index()];
    let mut probes = vec![(RadialRole::Containing, (outer[0] + outer[1]) / 2.0)];
    if inner[0] < outer[0] {
        probes.push((RadialRole::Contained, (inner[0] + outer[0]) / 2.0));
    }
    if inner[1] > outer[1] {
        probes.push((RadialRole::Contained, (outer[1] + inner[1]) / 2.0));
    }
    assert!(probes.len() == 2 || probes.len() == 3, "{label}");
    for (role, axial) in probes {
        let point = radial_axis_point(fixture.frame, role, axial);
        let classified = part
            .classify_point_in_body(ClassifyPointInBodyRequest::new(body.clone(), point))
            .unwrap()
            .into_result()
            .unwrap();
        assert_eq!(
            classified.verdict(),
            &kernel::PointBodyVerdict::Interior,
            "{label}: {role:?} axis point at {axial}"
        );
    }
    let global = [outer[0].min(inner[0]), outer[1].max(inner[1])];
    let margin = (global[1] - global[0]) / 4.0;
    for axial in [global[0] - margin, global[1] + margin] {
        let point = radial_axis_point(fixture.frame, RadialRole::Containing, axial);
        let classified = part
            .classify_point_in_body(ClassifyPointInBodyRequest::new(body.clone(), point))
            .unwrap()
            .into_result()
            .unwrap();
        assert_eq!(
            classified.verdict(),
            &kernel::PointBodyVerdict::Exterior,
            "{label}: result extends beyond axial union at {axial}"
        );
    }
}

fn export_components(fixture: &Fixture, bodies: &[BodyId]) -> Vec<Vec<u8>> {
    let part = fixture.session.part(fixture.part_id.clone()).unwrap();
    bodies
        .iter()
        .map(|body| {
            part.export_xt(ExportXtRequest::new(body.clone()))
                .unwrap()
                .into_result()
                .unwrap()
                .bytes()
                .to_vec()
        })
        .collect()
}

struct InternalEvidence {
    report: kernel::OperationReport,
    exports: Vec<Vec<u8>>,
    bodies: Vec<BodyId>,
}

#[allow(clippy::too_many_arguments)]
fn assert_internal_outcome(
    fixture: &mut Fixture,
    before: FixtureSignature,
    outcome: OperationOutcome<BooleanOutcome>,
    expected: ExpectedResult,
    intervals: [[f64; 2]; 2],
    containing_operand: usize,
    reversed_axes: [bool; 2],
    capture_exports: bool,
    label: &str,
) -> InternalEvidence {
    assert_eq!(
        internal_usage_at(&outcome, BOOLEAN_BSP_WORK, ResourceKind::Work),
        Some(INTERNAL_TANGENCY_RELATION_WORK),
        "{label}: relation work changed"
    );
    assert_eq!(
        internal_usage_at(&outcome, BOOLEAN_REALIZED_VERTICES, ResourceKind::Items),
        Some(match expected {
            ExpectedResult::TangentShoulder => 1,
            ExpectedResult::TangentChain => 2,
            _ => 0,
        }),
        "{label}: realized-vertex accounting changed"
    );
    let report = outcome.report().clone();
    let result = outcome.into_result().unwrap();
    let (exports, bodies) = match expected {
        ExpectedResult::Refused => {
            assert_eq!(
                internal_usage_at_report(&report, BOOLEAN_POST_SELECTION_WORK, ResourceKind::Work),
                Some(0),
                "{label}: refusal entered realization"
            );
            assert!(
                matches!(
                    result,
                    BooleanOutcome::Refused(BooleanRefusal::CurvedResultTopologyUnsupported)
                ),
                "{label}: unsupported tangent boundary returned {result:#?}"
            );
            assert_eq!(
                fixture_signature(fixture),
                before,
                "{label}: refusal mutated part"
            );
            assert_source_bodies_preserved(fixture, 2);
            (Vec::new(), Vec::new())
        }
        ExpectedResult::ProvenEmpty => {
            assert!(
                matches!(result, BooleanOutcome::Success(BooleanResult::ProvenEmpty)),
                "{label}: regularized empty result returned {result:#?}"
            );
            assert_eq!(
                fixture_signature(fixture),
                before,
                "{label}: empty result mutated part"
            );
            assert_source_bodies_preserved(fixture, 2);
            (Vec::new(), Vec::new())
        }
        ExpectedResult::SourceCopy(role) => {
            let BooleanOutcome::Success(BooleanResult::Created(created)) = result else {
                panic!("{label}: source-copy result returned {result:#?}")
            };
            assert_full_cylinder_components(fixture, &created, label);
            let source = role_body(fixture, containing_operand, role);
            assert_whole_source_copy_lineage(fixture, &created, source);
            let span = [intervals[role.index()]];
            assert_component_order_and_interiors(fixture, created.bodies(), role, &span, label);
            assert_source_bodies_preserved(fixture, 3);
            let exports = if capture_exports {
                export_components(fixture, created.bodies())
            } else {
                Vec::new()
            };
            (exports, created.bodies().to_vec())
        }
        ExpectedResult::RebuiltBands(spans) => {
            let BooleanOutcome::Success(BooleanResult::Created(created)) = result else {
                panic!("{label}: rebuilt-band result returned {result:#?}")
            };
            assert_eq!(created.bodies().len(), spans.len(), "{label}");
            assert_full_cylinder_components(fixture, &created, label);
            assert_rebuilt_band_lineage(
                fixture,
                &created,
                intervals,
                containing_operand,
                reversed_axes,
                spans,
                label,
            );
            assert_component_order_and_interiors(
                fixture,
                created.bodies(),
                RadialRole::Contained,
                spans,
                label,
            );
            assert_source_bodies_preserved(fixture, 2 + spans.len());
            let exports = if capture_exports {
                export_components(fixture, created.bodies())
            } else {
                Vec::new()
            };
            (exports, created.bodies().to_vec())
        }
        ExpectedResult::TangentShoulder => {
            let BooleanOutcome::Success(BooleanResult::Created(created)) = result else {
                panic!("{label}: tangent-shoulder result returned {result:#?}")
            };
            assert_full_tangent_shoulder(fixture, &created, label);
            assert_tangent_shoulder_lineage(
                fixture,
                &created,
                intervals,
                containing_operand,
                reversed_axes,
                label,
            );
            assert_tangent_union_interiors(fixture, created.bodies()[0].clone(), intervals, label);
            assert_source_bodies_preserved(fixture, 3);
            let exports = if capture_exports {
                export_components(fixture, created.bodies())
            } else {
                Vec::new()
            };
            (exports, created.bodies().to_vec())
        }
        ExpectedResult::TangentChain => {
            let BooleanOutcome::Success(BooleanResult::Created(created)) = result else {
                panic!("{label}: tangent-chain result returned {result:#?}")
            };
            assert_full_tangent_chain(fixture, &created, label);
            assert_tangent_chain_lineage(
                fixture,
                &created,
                containing_operand,
                reversed_axes,
                label,
            );
            assert_tangent_union_interiors(fixture, created.bodies()[0].clone(), intervals, label);
            assert_source_bodies_preserved(fixture, 3);
            let exports = if capture_exports {
                export_components(fixture, created.bodies())
            } else {
                Vec::new()
            };
            (exports, created.bodies().to_vec())
        }
    };
    InternalEvidence {
        report,
        exports,
        bodies,
    }
}

fn internal_usage_at_report(
    report: &kernel::OperationReport,
    stage: kernel::StageId,
    resource: ResourceKind,
) -> Option<u64> {
    report
        .usage()
        .iter()
        .find(|usage| usage.stage == stage && usage.resource == resource)
        .map(|usage| usage.consumed)
}

#[allow(clippy::too_many_arguments)]
fn execute_expected(
    fixture: &mut Fixture,
    request: InternalRequest,
    expected: ExpectedResult,
    intervals: [[f64; 2]; 2],
    containing_operand: usize,
    reversed_axes: [bool; 2],
    capture_exports: bool,
    label: &str,
) -> InternalEvidence {
    let before = fixture_signature(fixture);
    let outcome = run_internal_tangency(fixture, containing_operand, request);
    assert_internal_outcome(
        fixture,
        before,
        outcome,
        expected,
        intervals,
        containing_operand,
        reversed_axes,
        capture_exports,
        label,
    )
}

fn assert_same_internal_evidence(
    actual: &InternalEvidence,
    expected: &InternalEvidence,
    label: &str,
) {
    assert_eq!(
        actual.report, expected.report,
        "{label}: operation report changed"
    );
    assert_eq!(actual.exports.len(), expected.exports.len(), "{label}");
    assert_eq!(actual.bodies.len(), expected.bodies.len(), "{label}");
    for (actual, expected) in actual.exports.iter().zip(&expected.exports) {
        assert_xt_equal(actual, expected, label);
    }
}

#[test]
fn exact_internal_tangency_executes_the_regularized_axial_semantic_table() {
    for row in SEMANTIC_ROWS {
        for containing_operand in [0, 1] {
            for (request, expected) in [
                (InternalRequest::Intersect { swapped: false }, row.intersect),
                (InternalRequest::Unite { swapped: false }, row.unite),
                (
                    InternalRequest::ContainedMinusContaining,
                    row.contained_minus_containing,
                ),
                (
                    InternalRequest::ContainingMinusContained,
                    ExpectedResult::Refused,
                ),
            ] {
                let reversed_axes = [false; 2];
                let mut fixture = internal_tangency_fixture(
                    Placement::World,
                    row.intervals,
                    containing_operand,
                    reversed_axes,
                );
                let label = format!(
                    "{} containing_operand={containing_operand} request={request:?}",
                    row.name
                );
                let evidence = execute_expected(
                    &mut fixture,
                    request,
                    expected,
                    row.intervals,
                    containing_operand,
                    reversed_axes,
                    false,
                    &label,
                );
                if let ExpectedResult::RebuiltBands(spans) = expected {
                    assert_eq!(evidence.bodies.len(), spans.len(), "{label}");
                    for (body, span) in evidence.bodies.into_iter().zip(spans.iter().copied()) {
                        assert_internal_span_properties(&fixture, body, span);
                    }
                }
            }
        }
    }
}

fn exercise_deterministic_family(
    placement: Placement,
    row: SemanticRow,
    containing_operand: usize,
    reversed_axes: [bool; 2],
    requests: &[InternalRequest],
    expected: ExpectedResult,
) -> usize {
    let mut canonical = None;
    for (trial, request) in requests.iter().copied().enumerate() {
        let mut fixture =
            internal_tangency_fixture(placement, row.intervals, containing_operand, reversed_axes);
        let label = format!(
            "{} {placement:?} containing_operand={containing_operand} reversed={reversed_axes:?} request={request:?} trial={trial}",
            row.name,
        );
        let evidence = execute_expected(
            &mut fixture,
            request,
            expected,
            row.intervals,
            containing_operand,
            reversed_axes,
            true,
            &label,
        );
        if let Some(canonical) = canonical.as_ref() {
            assert_same_internal_evidence(&evidence, canonical, &label);
        } else {
            canonical = Some(evidence);
        }
    }
    requests.len()
}

#[test]
fn exact_internal_tangency_is_deterministic_across_frames_orders_and_axis_directions() {
    let mut executions = 0;
    for row in [
        CROSSING,
        CONTAINING_COVERS_CONTAINED,
        CONTAINED_COVERS_CONTAINING,
    ] {
        for placement in [Placement::World, Placement::Oblique] {
            for containing_operand in [0, 1] {
                for reversed_axes in [[false, false], [false, true], [true, false], [true, true]] {
                    executions += exercise_deterministic_family(
                        placement,
                        row,
                        containing_operand,
                        reversed_axes,
                        &[
                            InternalRequest::Intersect { swapped: false },
                            InternalRequest::Intersect { swapped: true },
                            InternalRequest::Intersect { swapped: false },
                        ],
                        row.intersect,
                    );
                    executions += exercise_deterministic_family(
                        placement,
                        row,
                        containing_operand,
                        reversed_axes,
                        &[
                            InternalRequest::Unite { swapped: false },
                            InternalRequest::Unite { swapped: true },
                            InternalRequest::Unite { swapped: false },
                        ],
                        row.unite,
                    );
                    executions += exercise_deterministic_family(
                        placement,
                        row,
                        containing_operand,
                        reversed_axes,
                        &[
                            InternalRequest::ContainedMinusContaining,
                            InternalRequest::ContainedMinusContaining,
                        ],
                        row.contained_minus_containing,
                    );
                    executions += exercise_deterministic_family(
                        placement,
                        row,
                        containing_operand,
                        reversed_axes,
                        &[
                            InternalRequest::ContainingMinusContained,
                            InternalRequest::ContainingMinusContained,
                        ],
                        ExpectedResult::Refused,
                    );
                }
            }
        }
    }
    assert_eq!(executions, 480);
}

#[test]
fn internal_tangency_refusals_roll_back_before_supported_replay() {
    let reversed_axes = [true, false];
    for containing_operand in [0, 1] {
        let mut after_subtract_refusal = internal_tangency_fixture(
            Placement::Oblique,
            CONTAINED_COVERS_CONTAINING.intervals,
            containing_operand,
            reversed_axes,
        );
        execute_expected(
            &mut after_subtract_refusal,
            InternalRequest::ContainingMinusContained,
            ExpectedResult::Refused,
            CONTAINED_COVERS_CONTAINING.intervals,
            containing_operand,
            reversed_axes,
            false,
            "pinched two-shoulder tangent-annulus refusal",
        );
        let replay = execute_expected(
            &mut after_subtract_refusal,
            InternalRequest::Unite { swapped: true },
            ExpectedResult::TangentChain,
            CONTAINED_COVERS_CONTAINING.intervals,
            containing_operand,
            reversed_axes,
            true,
            "post-subtract-refusal tangent-chain replay",
        );
        let mut baseline = internal_tangency_fixture(
            Placement::Oblique,
            CONTAINED_COVERS_CONTAINING.intervals,
            containing_operand,
            reversed_axes,
        );
        let baseline = execute_expected(
            &mut baseline,
            InternalRequest::Unite { swapped: true },
            ExpectedResult::TangentChain,
            CONTAINED_COVERS_CONTAINING.intervals,
            containing_operand,
            reversed_axes,
            true,
            "fresh tangent-chain baseline",
        );
        assert_same_internal_evidence(&replay, &baseline, "chain refusal rollback");

        let mut after_crossing_subtract_refusal = internal_tangency_fixture(
            Placement::Oblique,
            CROSSING.intervals,
            containing_operand,
            reversed_axes,
        );
        execute_expected(
            &mut after_crossing_subtract_refusal,
            InternalRequest::ContainingMinusContained,
            ExpectedResult::Refused,
            CROSSING.intervals,
            containing_operand,
            reversed_axes,
            false,
            "pinched tangent-annulus refusal",
        );
        let replay = execute_expected(
            &mut after_crossing_subtract_refusal,
            InternalRequest::ContainedMinusContaining,
            CROSSING.contained_minus_containing,
            CROSSING.intervals,
            containing_operand,
            reversed_axes,
            true,
            "post-subtract-refusal I-O replay",
        );
        let mut baseline = internal_tangency_fixture(
            Placement::Oblique,
            CROSSING.intervals,
            containing_operand,
            reversed_axes,
        );
        let baseline = execute_expected(
            &mut baseline,
            InternalRequest::ContainedMinusContaining,
            CROSSING.contained_minus_containing,
            CROSSING.intervals,
            containing_operand,
            reversed_axes,
            true,
            "fresh I-O baseline",
        );
        assert_same_internal_evidence(&replay, &baseline, "O-I refusal rollback");
    }
}

fn internal_settings_at(stage: kernel::StageId, allowed: u64) -> OperationSettings {
    OperationSettings::new().with_budget_overrides(
        BudgetPlan::new([LimitSpec::new(
            stage,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            allowed,
        )])
        .unwrap(),
    )
}

fn assert_internal_work_limit(
    outcome: &OperationOutcome<BooleanOutcome>,
    stage: kernel::StageId,
    expected_work: u64,
) {
    let limit = *outcome
        .report()
        .limit_events()
        .first()
        .expect("internal-tangency N-1 refusal recorded no limit event");
    assert_eq!(limit.stage, stage);
    assert_eq!(limit.resource, ResourceKind::Work);
    assert_eq!(limit.allowed, expected_work - 1);
    assert_eq!(limit.consumed, expected_work);
    assert_eq!(outcome.result().unwrap_err().limit(), Some(limit));
    assert_eq!(outcome.report().limit_events(), &[limit]);
}

fn assert_internal_realization_frontier(
    intervals: [[f64; 2]; 2],
    request: InternalRequest,
    expected: ExpectedResult,
    expected_work: u64,
) {
    let containing_operand = 0;
    let reversed_axes = [false; 2];

    let mut baseline = internal_tangency_fixture(
        Placement::World,
        intervals,
        containing_operand,
        reversed_axes,
    );
    let outcome = run_internal_tangency(&mut baseline, containing_operand, request);
    assert_eq!(
        internal_usage_at(&outcome, BOOLEAN_POST_SELECTION_WORK, ResourceKind::Work),
        Some(expected_work)
    );
    assert!(matches!(
        outcome.into_result().unwrap(),
        BooleanOutcome::Success(BooleanResult::Created(_))
    ));

    let mut admitted = internal_tangency_fixture(
        Placement::World,
        intervals,
        containing_operand,
        reversed_axes,
    );
    let outcome = run_internal_tangency_with_settings(
        &mut admitted,
        containing_operand,
        request,
        internal_settings_at(BOOLEAN_POST_SELECTION_WORK, expected_work),
    );
    assert_eq!(
        internal_usage_at(&outcome, BOOLEAN_POST_SELECTION_WORK, ResourceKind::Work),
        Some(expected_work)
    );
    assert!(matches!(
        outcome.into_result().unwrap(),
        BooleanOutcome::Success(BooleanResult::Created(_))
    ));

    let mut denied = internal_tangency_fixture(
        Placement::World,
        intervals,
        containing_operand,
        reversed_axes,
    );
    let before = fixture_signature(&denied);
    let outcome = run_internal_tangency_with_settings(
        &mut denied,
        containing_operand,
        request,
        internal_settings_at(BOOLEAN_POST_SELECTION_WORK, expected_work - 1),
    );
    assert_internal_work_limit(&outcome, BOOLEAN_POST_SELECTION_WORK, expected_work);
    assert_eq!(fixture_signature(&denied), before);
    assert_source_bodies_preserved(&denied, 2);

    let replay = run_internal_tangency(&mut denied, containing_operand, request);
    assert_internal_outcome(
        &mut denied,
        before,
        replay,
        expected,
        intervals,
        containing_operand,
        reversed_axes,
        false,
        "post-realization-limit replay",
    );
}

#[test]
fn internal_tangency_realization_work_has_exact_atomic_frontiers() {
    assert_internal_realization_frontier(
        CONTAINED_COVERS_CONTAINING.intervals,
        InternalRequest::Unite { swapped: true },
        ExpectedResult::TangentChain,
        INTERNAL_TANGENCY_CHAIN_WORK,
    );
    assert_internal_realization_frontier(
        CROSSING.intervals,
        InternalRequest::Unite { swapped: true },
        ExpectedResult::TangentShoulder,
        INTERNAL_TANGENCY_SHOULDER_WORK,
    );
    assert_internal_realization_frontier(
        CROSSING.intervals,
        InternalRequest::Intersect { swapped: false },
        CROSSING.intersect,
        INTERNAL_TANGENCY_BAND_WORK,
    );
    assert_internal_realization_frontier(
        [[-1.0, 1.0], [-2.0, 2.0]],
        InternalRequest::ContainedMinusContaining,
        ExpectedResult::RebuiltBands(&[[-2.0, -1.0], [1.0, 2.0]]),
        2 * INTERNAL_TANGENCY_BAND_WORK,
    );
    assert_internal_realization_frontier(
        [[-2.0, 2.0], [-1.0, 1.0]],
        InternalRequest::Intersect { swapped: true },
        ExpectedResult::SourceCopy(RadialRole::Contained),
        WHOLE_CYLINDER_COPY_WORK,
    );
}

#[test]
fn tangent_shoulder_properties_have_exact_frontier_and_independent_union_oracle() {
    let containing_operand = 1;
    let reversed_axes = [true, false];
    let mut fixture = internal_tangency_fixture(
        Placement::Oblique,
        CROSSING.intervals,
        containing_operand,
        reversed_axes,
    );
    let outcome = run_internal_tangency(
        &mut fixture,
        containing_operand,
        InternalRequest::Unite { swapped: true },
    );
    assert_eq!(
        internal_usage_at(&outcome, BOOLEAN_POST_SELECTION_WORK, ResourceKind::Work),
        Some(INTERNAL_TANGENCY_SHOULDER_WORK)
    );
    let result = outcome.into_result().unwrap();
    let BooleanOutcome::Success(BooleanResult::Created(created)) = result else {
        panic!("internal-tangency shoulder property fixture returned {result:#?}")
    };
    assert_full_tangent_shoulder(&fixture, &created, "shoulder property fixture");
    assert_tangent_shoulder_lineage(
        &fixture,
        &created,
        CROSSING.intervals,
        containing_operand,
        reversed_axes,
        "shoulder property fixture",
    );
    assert_tangent_union_properties(
        &fixture,
        created.bodies()[0].clone(),
        CROSSING.intervals,
        INTERNAL_TANGENCY_SHOULDER_PROPERTIES_WORK,
        "internal-tangency shoulder",
    );
    assert_source_bodies_preserved(&fixture, 3);
}

#[test]
fn tangent_chain_properties_have_exact_frontier_and_independent_union_oracle() {
    let containing_operand = 1;
    let reversed_axes = [true, false];
    let mut fixture = internal_tangency_fixture(
        Placement::Oblique,
        CONTAINED_COVERS_CONTAINING.intervals,
        containing_operand,
        reversed_axes,
    );
    let outcome = run_internal_tangency(
        &mut fixture,
        containing_operand,
        InternalRequest::Unite { swapped: true },
    );
    assert_eq!(
        internal_usage_at(&outcome, BOOLEAN_POST_SELECTION_WORK, ResourceKind::Work),
        Some(INTERNAL_TANGENCY_CHAIN_WORK)
    );
    let result = outcome.into_result().unwrap();
    let BooleanOutcome::Success(BooleanResult::Created(created)) = result else {
        panic!("internal-tangency chain property fixture returned {result:#?}")
    };
    assert_full_tangent_chain(&fixture, &created, "chain property fixture");
    assert_tangent_chain_lineage(
        &fixture,
        &created,
        containing_operand,
        reversed_axes,
        "chain property fixture",
    );
    assert_tangent_union_properties(
        &fixture,
        created.bodies()[0].clone(),
        CONTAINED_COVERS_CONTAINING.intervals,
        INTERNAL_TANGENCY_CHAIN_PROPERTIES_WORK,
        "internal-tangency chain",
    );
    assert_source_bodies_preserved(&fixture, 3);
}

#[test]
fn rebuilt_internal_tangent_band_properties_have_exact_frontiers_and_cylinder_oracles() {
    let intervals = [[-1.0, 1.0], [-2.0, 2.0]];
    let containing_operand = 1;
    let reversed_axes = [true, false];
    let spans = [[-2.0, -1.0], [1.0, 2.0]];
    let mut fixture = internal_tangency_fixture(
        Placement::World,
        intervals,
        containing_operand,
        reversed_axes,
    );
    let outcome = run_internal_tangency(
        &mut fixture,
        containing_operand,
        InternalRequest::ContainedMinusContaining,
    );
    assert_eq!(
        internal_usage_at(&outcome, BOOLEAN_POST_SELECTION_WORK, ResourceKind::Work),
        Some(2 * INTERNAL_TANGENCY_BAND_WORK)
    );
    let result = outcome.into_result().unwrap();
    let BooleanOutcome::Success(BooleanResult::Created(created)) = result else {
        panic!("internal-tangency property fixture returned {result:#?}")
    };
    assert_full_cylinder_components(&fixture, &created, "property fixture");
    assert_rebuilt_band_lineage(
        &fixture,
        &created,
        intervals,
        containing_operand,
        reversed_axes,
        &spans,
        "property fixture",
    );
    for (body, span) in created.bodies().iter().cloned().zip(spans) {
        assert_internal_span_properties(&fixture, body, span);
    }
    assert_source_bodies_preserved(&fixture, 4);
}

#[test]
fn rebuilt_internal_tangent_results_have_stable_xt_and_fast_self_import_twice() {
    for (name, intervals, request, expected) in [
        (
            "contained-radius band",
            CROSSING.intervals,
            InternalRequest::Intersect { swapped: true },
            CROSSING.intersect,
        ),
        (
            "tangent shoulder",
            CROSSING.intervals,
            InternalRequest::Unite { swapped: true },
            ExpectedResult::TangentShoulder,
        ),
        (
            "two-shoulder tangent chain",
            CONTAINED_COVERS_CONTAINING.intervals,
            InternalRequest::Unite { swapped: true },
            ExpectedResult::TangentChain,
        ),
    ] {
        let containing_operand = 1;
        let reversed_axes = [false, true];
        let mut fixture = internal_tangency_fixture(
            Placement::Oblique,
            intervals,
            containing_operand,
            reversed_axes,
        );
        let evidence = execute_expected(
            &mut fixture,
            request,
            expected,
            intervals,
            containing_operand,
            reversed_axes,
            true,
            name,
        );
        let [body] = evidence.bodies.as_slice() else {
            panic!("{name}: expected exactly one exported result")
        };
        let [first] = evidence.exports.as_slice() else {
            panic!("{name}: expected exactly one X_T payload")
        };
        let part = fixture.session.part(fixture.part_id.clone()).unwrap();
        let second = part
            .export_xt(ExportXtRequest::new(body.clone()))
            .unwrap()
            .into_result()
            .unwrap()
            .bytes()
            .to_vec();
        assert_xt_equal(
            first,
            &second,
            &format!("{name}: repeat X_T export changed"),
        );
        assert_fast_self_import(&mut fixture.session, first);
        assert_fast_self_import(&mut fixture.session, first);
    }
}

#[derive(Debug, Clone, Copy)]
struct RadialBoundaryAdversary {
    name: &'static str,
    radii: [f64; 2],
    center_separation: f64,
}

fn radial_adversary_settings(loose: bool) -> OperationSettings {
    if loose {
        OperationSettings::new().with_tolerances(Tolerances::with_linear(1.0e-6).unwrap())
    } else {
        OperationSettings::new()
    }
}

fn exact_tolerance_evidence(
    containing_operand: usize,
    intervals: [[f64; 2]; 2],
    request: InternalRequest,
    expected: ExpectedResult,
    settings: OperationSettings,
    label: &str,
) -> InternalEvidence {
    let reversed_axes = [false; 2];
    let mut fixture = internal_tangency_fixture(
        Placement::World,
        intervals,
        containing_operand,
        reversed_axes,
    );
    let before = fixture_signature(&fixture);
    let outcome =
        run_internal_tangency_with_settings(&mut fixture, containing_operand, request, settings);
    assert_internal_outcome(
        &mut fixture,
        before,
        outcome,
        expected,
        intervals,
        containing_operand,
        reversed_axes,
        true,
        label,
    )
}

fn assert_radial_neighbor_refusal(
    adversary: RadialBoundaryAdversary,
    containing_operand: usize,
    loose: bool,
) {
    let reversed_axes = [false; 2];
    let mut fixture = internal_tangency_fixture_with_radial_geometry(
        Placement::World,
        CROSSING.intervals,
        containing_operand,
        reversed_axes,
        adversary.radii,
        adversary.center_separation,
    );
    let before = fixture_signature(&fixture);
    let outcome = run_internal_tangency_with_settings(
        &mut fixture,
        containing_operand,
        InternalRequest::Intersect { swapped: true },
        radial_adversary_settings(loose),
    );
    let label = format!(
        "{} containing_operand={containing_operand} loose={loose}",
        adversary.name
    );
    assert_eq!(
        internal_usage_at(&outcome, BOOLEAN_BSP_WORK, ResourceKind::Work),
        Some(INTERNAL_TANGENCY_RELATION_WORK),
        "{label}"
    );
    assert_eq!(
        internal_usage_at(&outcome, BOOLEAN_POST_SELECTION_WORK, ResourceKind::Work),
        Some(0),
        "{label}: near boundary entered realization"
    );
    assert_eq!(
        internal_usage_at(&outcome, BOOLEAN_REALIZED_VERTICES, ResourceKind::Items),
        Some(0),
        "{label}: near boundary allocated vertices"
    );
    let result = outcome.into_result().unwrap();
    assert!(
        matches!(result, BooleanOutcome::Refused(_)),
        "{label}: near boundary acquired the exact internal-tangency contract: {result:#?}"
    );
    assert_eq!(
        fixture_signature(&fixture),
        before,
        "{label}: refusal mutated the part"
    );
    assert_source_bodies_preserved(&fixture, 2);
}

#[test]
fn exact_internal_tangency_is_tolerance_independent_and_not_inferred_across_one_ulp() {
    for containing_operand in [0, 1] {
        for (name, intervals, request, expected) in [
            (
                "Intersect",
                CROSSING.intervals,
                InternalRequest::Intersect { swapped: true },
                CROSSING.intersect,
            ),
            (
                "one-shoulder Unite",
                CROSSING.intervals,
                InternalRequest::Unite { swapped: true },
                ExpectedResult::TangentShoulder,
            ),
            (
                "two-shoulder Unite",
                CONTAINED_COVERS_CONTAINING.intervals,
                InternalRequest::Unite { swapped: true },
                ExpectedResult::TangentChain,
            ),
        ] {
            let baseline = exact_tolerance_evidence(
                containing_operand,
                intervals,
                request,
                expected,
                radial_adversary_settings(false),
                &format!("exact internal tangency {name} baseline"),
            );
            let loose = exact_tolerance_evidence(
                containing_operand,
                intervals,
                request,
                expected,
                radial_adversary_settings(true),
                &format!("exact internal tangency {name} loose tolerance"),
            );
            assert_same_internal_evidence(
                &loose,
                &baseline,
                &format!("loose tolerance changed exact internal tangency {name}"),
            );
        }
    }

    let adversaries = [
        RadialBoundaryAdversary {
            name: "center separation one ULP inward",
            radii: [CONTAINING_RADIUS, CONTAINED_RADIUS],
            center_separation: CENTER_SEPARATION.next_down(),
        },
        RadialBoundaryAdversary {
            name: "center separation one ULP outward",
            radii: [CONTAINING_RADIUS, CONTAINED_RADIUS],
            center_separation: CENTER_SEPARATION.next_up(),
        },
        RadialBoundaryAdversary {
            name: "containing radius one ULP smaller",
            radii: [CONTAINING_RADIUS.next_down(), CONTAINED_RADIUS],
            center_separation: CENTER_SEPARATION,
        },
        RadialBoundaryAdversary {
            name: "containing radius one ULP larger",
            radii: [CONTAINING_RADIUS.next_up(), CONTAINED_RADIUS],
            center_separation: CENTER_SEPARATION,
        },
        RadialBoundaryAdversary {
            name: "contained radius one ULP smaller",
            radii: [CONTAINING_RADIUS, CONTAINED_RADIUS.next_down()],
            center_separation: CENTER_SEPARATION,
        },
        RadialBoundaryAdversary {
            name: "contained radius one ULP larger",
            radii: [CONTAINING_RADIUS, CONTAINED_RADIUS.next_up()],
            center_separation: CENTER_SEPARATION,
        },
    ];
    for adversary in adversaries {
        for containing_operand in [0, 1] {
            for loose in [false, true] {
                assert_radial_neighbor_refusal(adversary, containing_operand, loose);
            }
        }
    }
}
