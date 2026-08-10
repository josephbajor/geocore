"""Host-native Parasolid probes that preserve raw there-and-back evidence."""

from __future__ import annotations

import hashlib
import json
import math
import re
from collections import Counter, defaultdict
from dataclasses import asdict
from pathlib import Path

from .onshape import (
    OracleError,
    add_part_studio_feature,
    create_feature_studio,
    create_part_studio,
    delete_element,
    find_translated_part_studio,
    get_feature_studio_contents,
    get_feature_studio_specs,
    get_part_studio_body_details,
    reexport_element,
    run_fixture,
    update_feature_studio_contents,
)

PROBE_NAME = "corner-contact-subtract"
SOURCE_PATH = Path(__file__).with_name("probes") / "corner_contact_subtract.fs"
NATIVE_XT = "corner_contact_subtract_native.x_t"
ROUNDTRIP_XT = "corner_contact_subtract_roundtrip.x_t"
CONTACT_POINTS = ((5.0, -12.0, 16.0), (5.0, 12.0, 16.0))
POINT_TOLERANCE = 1.0e-8


class _UnionFind:
    def __init__(self, size):
        self.parent = list(range(size))

    def find(self, value):
        while self.parent[value] != value:
            self.parent[value] = self.parent[self.parent[value]]
            value = self.parent[value]
        return value

    def union(self, first, second):
        first = self.find(first)
        second = self.find(second)
        if first != second:
            self.parent[second] = first

    def component_sizes(self):
        counts = Counter(self.find(value) for value in range(len(self.parent)))
        return sorted(counts.values(), reverse=True)


def _write_json(path, value):
    Path(path).write_text(
        json.dumps(value, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _point(vertex):
    point = vertex.get("point") or {}
    try:
        coordinates = (float(point["x"]), float(point["y"]), float(point["z"]))
    except (KeyError, TypeError, ValueError):
        raise OracleError("body details contained a vertex without a finite point")
    if not all(math.isfinite(value) for value in coordinates):
        raise OracleError("body details contained a vertex without a finite point")
    return coordinates


def _rounded_point(point):
    return [round(value, 12) for value in point]


def _near(first, second, tolerance=POINT_TOLERANCE):
    return math.dist(first, second) <= tolerance


def _histogram(values):
    return dict(sorted(Counter(values).items()))


def _body_summary(body):
    faces = body.get("faces") or []
    edges = body.get("edges") or []
    vertices = body.get("vertices") or []
    edge_by_id = {edge.get("id"): edge for edge in edges if edge.get("id")}
    edge_faces = defaultdict(set)
    vertex_faces = defaultdict(set)
    loop_count = 0
    coedge_count = 0

    for face_index, face in enumerate(faces):
        for loop in face.get("loops") or []:
            loop_count += 1
            for coedge in loop.get("coedges") or []:
                coedge_count += 1
                edge_id = coedge.get("edgeId")
                if edge_id:
                    edge_faces[edge_id].add(face_index)
                    for vertex_id in (edge_by_id.get(edge_id) or {}).get("vertices") or []:
                        vertex_faces[vertex_id].add(face_index)

    edge_components = _UnionFind(len(faces))
    for incident in edge_faces.values():
        incident = sorted(incident)
        for face_index in incident[1:]:
            edge_components.union(incident[0], face_index)

    vertex_components = _UnionFind(len(faces))
    for incident in vertex_faces.values():
        incident = sorted(incident)
        for face_index in incident[1:]:
            vertex_components.union(incident[0], face_index)

    points = sorted(_rounded_point(_point(vertex)) for vertex in vertices)
    return {
        "type": body.get("type", "UNKNOWN"),
        "faces": len(faces),
        "loops": loop_count,
        "coedges": coedge_count,
        "edges": len(edges),
        "vertices": len(vertices),
        "surface_types": _histogram(
            (face.get("surface") or {}).get("type", "UNKNOWN") for face in faces
        ),
        "curve_types": _histogram(
            (edge.get("curve") or {}).get("type", "UNKNOWN") for edge in edges
        ),
        "edge_face_component_sizes": edge_components.component_sizes(),
        "vertex_face_component_sizes": vertex_components.component_sizes(),
        "vertex_points_m": points,
    }


def _point_clusters(points):
    clusters = []
    for body_index, point in sorted(points, key=lambda item: item[1]):
        for cluster in clusters:
            if _near(point, cluster["point"]):
                cluster["members"].append((body_index, point))
                break
        else:
            clusters.append({"point": point, "members": [(body_index, point)]})
    return clusters


def _multiplicities(members):
    counts = Counter(body_index for body_index, _ in members)
    return sorted(counts.values(), reverse=True)


def summarize_body_details(details):
    """Normalize host topology without assuming the representation outcome.

    Host entity ids and ordering are intentionally removed. Body count,
    topological face connectivity, and coincident-but-distinct vertex
    multiplicities remain observable and stable across X_T replay.
    """
    bodies = details.get("bodies") or []
    summaries = [_body_summary(body) for body in bodies]
    summaries.sort(key=lambda value: json.dumps(value, sort_keys=True))

    all_points = []
    for body_index, body in enumerate(bodies):
        for vertex in body.get("vertices") or []:
            all_points.append((body_index, _point(vertex)))
    clusters = _point_clusters(all_points)
    coincident = [
        {
            "point_m": _rounded_point(cluster["point"]),
            "multiplicity": len(cluster["members"]),
            "per_body_multiplicities": _multiplicities(cluster["members"]),
        }
        for cluster in clusters
        if len(cluster["members"]) > 1
    ]
    coincident.sort(key=lambda value: value["point_m"])

    contact_vertices = []
    for contact in CONTACT_POINTS:
        members = [item for item in all_points if _near(item[1], contact)]
        contact_vertices.append(
            {
                "point_m": list(contact),
                "multiplicity": len(members),
                "per_body_multiplicities": _multiplicities(members),
            }
        )

    return {
        "error_enum": details.get("errorEnum", ""),
        "body_count": len(bodies),
        "bodies": summaries,
        "coincident_vertex_clusters": coincident,
        "expected_contact_vertices": contact_vertices,
    }


def render_feature_source(template_contents, source_path=SOURCE_PATH):
    """Render the probe at the exact library version chosen by the host."""
    match = re.search(r"FeatureScript\s+(\d+)\s*;", template_contents or "")
    if match is None:
        raise OracleError("Feature Studio template omitted its FeatureScript version")
    source = Path(source_path).read_text(encoding="utf-8")
    rendered = source.replace("__LIBRARY_VERSION__", match.group(1))
    if "__LIBRARY_VERSION__" in rendered:
        raise OracleError("host-probe FeatureScript version token was not fully rendered")
    return rendered


def _probe_feature(spec):
    namespace = spec.get("namespace")
    feature_type = spec.get("featureType")
    if not namespace or not feature_type:
        raise OracleError("corner-contact Feature Studio spec omitted its callable identity")
    return {
        "btType": "BTMFeature-134",
        "featureType": feature_type,
        "namespace": namespace,
        "name": "Corner contact subtract oracle",
        "parameters": [],
        "returnAfterSubfeatures": False,
        "suppressed": False,
    }


def _matching_spec(response):
    matches = [
        spec
        for spec in response.get("featureSpecs") or []
        if spec.get("featureType") == "cornerContactSubtract"
    ]
    if len(matches) != 1:
        raise OracleError(
            "corner-contact Feature Studio exported {} matching specs".format(len(matches))
        )
    return matches[0]


def _manifest(output, revision, status, source, extra=None):
    files = {}
    for path in sorted(output.iterdir()):
        if path.is_file() and path.name != "manifest.json":
            files[path.name] = {"bytes": path.stat().st_size, "sha256": _sha256(path)}
    value = {
        "schema_version": "kernel-oracle-host-probe.v1",
        "probe": PROBE_NAME,
        "source_revision": revision,
        "status": status,
        "feature_source_sha256": hashlib.sha256(source.encode("utf-8")).hexdigest(),
        "files": files,
    }
    if extra:
        value.update(extra)
    _write_json(output / "manifest.json", value)


def run_corner_contact_subtract_probe(transport, config, output_dir, revision="-"):
    """Run host-native Subtract, X_T replay it, and retain structural evidence.

    Returns 0 for a stable successful replay and 1 for a completed oracle
    finding (operation refusal, translation rejection, or topology change).
    Operational failures raise ``OracleError`` and are reported as exit 2 by
    the shared CLI.
    """
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    if any(output.iterdir()):
        raise OracleError("host-probe output directory must be empty: {}".format(output))

    source = ""
    created_elements = []
    cleanup_errors = []
    try:
        translated_part_studio = find_translated_part_studio(transport, config)

        feature_studio = create_feature_studio(
            transport, config, "corner-contact-oracle-feature"
        )
        feature_studio_id = feature_studio.get("id")
        if not feature_studio_id:
            raise OracleError("Feature Studio creation returned no element id")
        created_elements.append(feature_studio_id)

        template = get_feature_studio_contents(transport, config, feature_studio_id)
        source = render_feature_source(template.get("contents", ""))
        (output / "probe.fs").write_text(source, encoding="utf-8")
        update = update_feature_studio_contents(
            transport, config, feature_studio_id, template, source
        )
        _write_json(output / "feature-studio-update.json", update)
        spec = _matching_spec(
            get_feature_studio_specs(transport, config, feature_studio_id)
        )

        part_studio = create_part_studio(transport, config, "corner-contact-oracle-result")
        part_studio_id = part_studio.get("id")
        if not part_studio_id:
            raise OracleError("Part Studio creation returned no element id")
        created_elements.append(part_studio_id)

        feature_response = add_part_studio_feature(
            transport,
            config,
            part_studio_id,
            _probe_feature(spec),
        )
        _write_json(output / "feature-response.json", feature_response)
        feature_status = (feature_response.get("featureState") or {}).get(
            "featureStatus", ""
        )
        if feature_status != "OK":
            _manifest(
                output,
                revision,
                "operation-refused",
                source,
                {"feature_status": feature_status},
            )
            return 1

        native_details = get_part_studio_body_details(transport, config, part_studio_id)
        native_summary = summarize_body_details(native_details)
        _write_json(output / "native-body-details.json", native_details)
        _write_json(output / "native-summary.json", native_summary)
        native_path = output / NATIVE_XT
        native_path.write_bytes(reexport_element(transport, config, part_studio_id))

        translation = run_fixture(transport, config, native_path)
        _write_json(output / "roundtrip-translation.json", asdict(translation))
        if not translation.accepted:
            _manifest(
                output,
                revision,
                "roundtrip-import-rejected",
                source,
                {"native_summary": native_summary},
            )
            return 1

        replay_part_studio = (
            translation.result_element_ids[0]
            if translation.result_element_ids
            else translated_part_studio
        )
        replay_details = get_part_studio_body_details(
            transport, config, replay_part_studio
        )
        replay_summary = summarize_body_details(replay_details)
        _write_json(output / "roundtrip-body-details.json", replay_details)
        _write_json(output / "roundtrip-summary.json", replay_summary)
        (output / ROUNDTRIP_XT).write_bytes(
            reexport_element(transport, config, replay_part_studio)
        )

        stable = native_summary == replay_summary
        _manifest(
            output,
            revision,
            "stable" if stable else "topology-changed-on-roundtrip",
            source,
            {
                "topology_stable": stable,
                "native_summary": native_summary,
                "roundtrip_summary": replay_summary,
            },
        )
        return 0 if stable else 1
    finally:
        for element_id in reversed(created_elements):
            try:
                delete_element(transport, config, element_id)
            except OracleError as error:
                cleanup_errors.append(str(error))
        _write_json(
            output / "cleanup.json",
            {"created": len(created_elements), "errors": cleanup_errors},
        )
        if cleanup_errors:
            raise OracleError("host-probe cleanup failed: {}".format("; ".join(cleanup_errors)))
