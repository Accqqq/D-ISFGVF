#!/usr/bin/env python3
"""Offline Stage-2 V2 diagnostics/replay oracle.

This file intentionally has no ROS, subprocess, launch, network, simulator,
or workspace-binary dependency.  It consumes source-recorded canonical CSV
rows and checks identity/order/count conservation without converting 64-bit
identities through floating point.
"""

import argparse
import csv
import hashlib
import io
import math
import sys


SCHEMA_VERSION = "tube_v2_diagnostics_v1"
REQUIRED_FIELDS = (
    "schema_version", "record_kind", "ordinal", "capability_state",
    "planner_only", "has_completion", "completion_status",
    "completion_purpose", "completion_request_id",
    "completion_execution_generation", "completion_accepted_state_demand",
    "build_success",
    "build_path_cell_query_count", "build_failed_path_cell_query_count",
    "build_query_count", "build_failed_query_count",
    "build_child_query_count", "build_cell_count",
    "build_scheduled_cell_count", "build_accepted_cell_count",
    "build_witness_count", "build_max_depth_observed",
    "build_query_budget_reached", "build_cell_budget_reached",
    "build_witness_budget_reached", "build_sample_budget_reached",
    "event_type", "event_timestamp_ticks", "event_purpose",
    "event_request_id", "event_execution_generation",
    "event_accepted_state_demand", "stats_delivery_count",
    "stats_event_log_dropped", "metric_closed_inflated_voxel_volume",
    "metric_epsilon_present", "metric_epsilon", "metric_epsilon_unchanged",
    "metric_inflation_axes_present", "metric_inflation_x",
    "metric_inflation_y", "metric_inflation_z",
    "metric_inflation_provenance",
    "metric_observed_obstacle_containment_proven",
    "metric_vehicle_tracking_budget_proven",
    "metric_physical_containment_proven", "metric_support_complete",
    "boundary_live_k_present",
)
INTEGER_FIELDS = {
    "ordinal", "completion_request_id", "completion_execution_generation",
    "completion_accepted_state_demand", "event_timestamp_ticks",
    "event_request_id", "event_execution_generation",
    "event_accepted_state_demand", "stats_delivery_count",
    "stats_event_log_dropped", "boundary_index",
    "build_path_cell_query_count", "build_failed_path_cell_query_count",
    "build_query_count", "build_failed_query_count",
    "build_child_query_count", "build_cell_count",
    "build_scheduled_cell_count", "build_accepted_cell_count",
    "build_witness_count", "build_max_depth_observed",
}
BOOLEAN_FIELDS = {
    "planner_only", "has_completion", "completion_present",
    "build_success",
    "event_heavy_build_entered", "metric_closed_inflated_voxel_volume",
    "metric_epsilon_present", "metric_epsilon_unchanged",
    "metric_inflation_axes_present", "metric_physical_containment_proven",
    "metric_observed_obstacle_containment_proven",
    "metric_vehicle_tracking_budget_proven", "metric_support_complete",
    "boundary_live_k_present", "build_query_budget_reached",
    "build_cell_budget_reached", "build_witness_budget_reached",
    "build_sample_budget_reached",
}
FLOAT_FIELDS = {
    "metric_epsilon", "metric_inflation_x", "metric_inflation_y",
    "metric_inflation_z",
}


class ReplayError(ValueError):
    pass


def _integer(row, field):
    value = row.get(field, "")
    if value in ("", "not_applicable", "absent"):
        return None
    if value.startswith("+") or value.startswith("-") and value == "-0":
        # Decimal identities are deliberately canonical and unsigned where
        # the schema declares them unsigned.
        raise ReplayError("noncanonical integer in %s" % field)
    if not value.isdigit():
        raise ReplayError("invalid integer in %s" % field)
    return int(value)


def _boolean(row, field):
    value = row.get(field, "")
    if value in ("", "not_applicable", "absent"):
        return None
    if value == "true":
        return True
    if value == "false":
        return False
    raise ReplayError("invalid boolean in %s" % field)


def _number(row, field):
    value = row.get(field, "")
    if value in ("", "not_applicable", "absent"):
        return None
    try:
        parsed = float(value)
    except ValueError:
        raise ReplayError("invalid number in %s" % field)
    if not math.isfinite(parsed):
        raise ReplayError("nonfinite number in %s" % field)
    return parsed


def parse_trace(text):
    stream = io.StringIO(text)
    reader = csv.DictReader(stream)
    if not reader.fieldnames:
        raise ReplayError("missing CSV header")
    if len(set(reader.fieldnames)) != len(reader.fieldnames):
        raise ReplayError("duplicate CSV header name")
    missing = [name for name in REQUIRED_FIELDS if name not in reader.fieldnames]
    if missing:
        raise ReplayError("missing schema fields: %s" % ",".join(missing))
    rows = []
    for row_number, row in enumerate(reader, 2):
        if None in row:
            raise ReplayError("extra CSV columns at line %d" % row_number)
        if row.get("schema_version") != SCHEMA_VERSION:
            raise ReplayError("schema version mismatch at line %d" % row_number)
        for field in INTEGER_FIELDS:
            _integer(row, field)
        for field in BOOLEAN_FIELDS:
            _boolean(row, field)
        for field in FLOAT_FIELDS:
            _number(row, field)
        rows.append(row)
    if not rows:
        raise ReplayError("trace has no records")
    return reader.fieldnames, rows


def replay_trace(text):
    """Validate and return a byte-stable canonical replay of *text*."""
    fieldnames, rows = parse_trace(text)
    snapshots = [row for row in rows if row.get("record_kind") == "SNAPSHOT"]
    events = [row for row in rows if row.get("record_kind") == "EVENT"]
    boundaries = [row for row in rows if row.get("record_kind") == "BOUNDARY"]
    if len(snapshots) != 1:
        raise ReplayError("expected exactly one SNAPSHOT row")
    snapshot = snapshots[0]
    unknown = [row for row in rows
               if row.get("record_kind") not in ("SNAPSHOT", "EVENT", "BOUNDARY")]
    if unknown:
        raise ReplayError("unknown record kind")
    expected_kinds = (["SNAPSHOT"] + ["EVENT"] * len(events) +
                      ["BOUNDARY"] * len(boundaries))
    if [row.get("record_kind") for row in rows] != expected_kinds:
        raise ReplayError("record-kind order mismatch")

    event_ordinals = [_integer(row, "ordinal") for row in events]
    if event_ordinals != list(range(len(events))):
        raise ReplayError("event reorder, duplicate, or dropped ordinal")
    boundary_ordinals = [_integer(row, "boundary_index") for row in boundaries]
    if boundary_ordinals != list(range(len(boundaries))):
        raise ReplayError("boundary reorder, duplicate, or dropped ordinal")

    # Preserve exact purpose/order identities.  A build-finished event must
    # refer to the same request/generation as the supplied completion.
    completion_status = snapshot.get("completion_status", "")
    completion_id = _integer(snapshot, "completion_request_id")
    completion_generation = _integer(snapshot, "completion_execution_generation")
    completion_purpose = snapshot.get("completion_purpose", "")
    if completion_status in ("FAILED", "CANCELLED", "STALE") and \
            snapshot.get("capability_state") == "CERTIFIED_PROFILE":
        raise ReplayError("failed/cancelled/stale row is certified")
    if snapshot.get("capability_state") == "CERTIFIED_PROFILE" and \
            (completion_status != "BUILT" or
             not _boolean(snapshot, "build_success")):
        raise ReplayError("certified capability lacks BUILT completion")
    path_queries = _integer(snapshot, "build_path_cell_query_count")
    failed_path_queries = _integer(snapshot,
                                    "build_failed_path_cell_query_count")
    queries = _integer(snapshot, "build_query_count")
    failed_queries = _integer(snapshot, "build_failed_query_count")
    cells = _integer(snapshot, "build_cell_count")
    scheduled_cells = _integer(snapshot, "build_scheduled_cell_count")
    accepted_cells = _integer(snapshot, "build_accepted_cell_count")
    max_depth = _integer(snapshot, "build_max_depth_observed")
    if (path_queries is not None and failed_path_queries is not None and
            failed_path_queries > path_queries):
        raise ReplayError("failed path queries exceed path queries")
    if (queries is not None and failed_queries is not None and
            failed_queries > queries):
        raise ReplayError("failed queries exceed queries")
    if (cells is not None and scheduled_cells is not None and
            cells > scheduled_cells):
        raise ReplayError("visited cells exceed scheduled cells")
    if (accepted_cells is not None and cells is not None and
            accepted_cells > cells):
        raise ReplayError("accepted cells exceed visited cells")
    if max_depth is not None and max_depth < 0:
        raise ReplayError("negative maximum build depth")
    delivered_events = 0
    previous_timestamp = None
    purposes = []
    for row in events:
        timestamp = _integer(row, "event_timestamp_ticks")
        if timestamp is not None and previous_timestamp is not None and \
                timestamp < previous_timestamp:
            raise ReplayError("event timing is not monotonic")
        if timestamp is not None:
            previous_timestamp = timestamp
        event_type = row.get("event_type", "")
        if event_type == "COMPLETION_DELIVERED":
            delivered_events += 1
        purpose = row.get("event_purpose", "")
        if purpose not in ("CURRENT", "SUCCESSOR"):
            raise ReplayError("unknown purpose")
        purposes.append(purpose)
        if event_type in ("BUILD_STARTED", "BUILD_FINISHED",
                          "COMPLETION_DELIVERED") and \
                purpose == completion_purpose and completion_id is not None:
            if _integer(row, "event_request_id") != completion_id or \
                    _integer(row, "event_execution_generation") != \
                    completion_generation:
                raise ReplayError("completion/event identity mismatch")
    delivery_count = _integer(snapshot, "stats_delivery_count")
    dropped = _integer(snapshot, "stats_event_log_dropped") or 0
    if delivery_count is not None and delivered_events > delivery_count:
        raise ReplayError("event delivery count exceeds worker stats")
    if dropped == 0 and delivery_count is not None and delivered_events != delivery_count:
        raise ReplayError("delivery count not conserved")
    for row in boundaries:
        if _boolean(row, "boundary_live_k_present"):
            raise ReplayError("live K is not applicable in Stage 2")
    epsilon_present = _boolean(snapshot, "metric_epsilon_present")
    if epsilon_present and _number(snapshot, "metric_epsilon") is None:
        raise ReplayError("epsilon marked present without a value")
    axes_present = _boolean(snapshot, "metric_inflation_axes_present")
    axes = [_number(snapshot, field) for field in
            ("metric_inflation_x", "metric_inflation_y", "metric_inflation_z")]
    if axes_present and any(value is None for value in axes):
        raise ReplayError("inflation axes marked present without values")
    provenance = snapshot.get("metric_inflation_provenance", "")
    if axes_present and provenance in ("", "not_applicable", "absent"):
        raise ReplayError("inflation axes lack provenance")
    if _boolean(snapshot, "metric_physical_containment_proven") and \
            not (_boolean(snapshot, "metric_closed_inflated_voxel_volume") and
                 axes_present and _boolean(snapshot, "metric_support_complete") and
                 _boolean(snapshot,
                          "metric_observed_obstacle_containment_proven") and
                 _boolean(snapshot, "metric_vehicle_tracking_budget_proven")):
        raise ReplayError("physical containment lacks metric axes/support/evidence")
    # Re-emit with the original field order and csv.writer so replay output is
    # deterministic while preserving all decimal 64-bit identity text.
    out = io.StringIO(newline="")
    writer = csv.DictWriter(out, fieldnames=fieldnames, lineterminator="\n")
    writer.writeheader()
    writer.writerows(rows)
    return out.getvalue()


def _fixture():
    # Small deterministic source-recorded trace.  Its identities exceed the
    # exact integer range of IEEE-754 doubles to guard against lossy replay.
    fields = list(REQUIRED_FIELDS)
    fields.extend(["completion_present", "event_heavy_build_entered",
                   "boundary_index"])
    # Keep header names unique while adding fixture-only fields.
    fields = list(dict.fromkeys(fields))
    snapshot = {field: "not_applicable" for field in fields}
    snapshot.update({
        "schema_version": SCHEMA_VERSION, "record_kind": "SNAPSHOT",
        "ordinal": "0", "capability_state": "CERTIFIED_PROFILE",
        "planner_only": "true", "has_completion": "true",
        "completion_present": "true", "completion_status": "BUILT",
        "completion_purpose": "CURRENT", "completion_request_id": "8589934592",
        "completion_execution_generation": "4294967296",
        "completion_accepted_state_demand": "25769803776",
        "build_success": "true",
        "build_path_cell_query_count": "13",
        "build_failed_path_cell_query_count": "1",
        "build_query_count": "17", "build_failed_query_count": "2",
        "build_child_query_count": "5", "build_cell_count": "19",
        "build_scheduled_cell_count": "21",
        "build_accepted_cell_count": "18", "build_witness_count": "23",
        "build_max_depth_observed": "4",
        "build_query_budget_reached": "false",
        "build_cell_budget_reached": "false",
        "build_witness_budget_reached": "false",
        "build_sample_budget_reached": "false",
        "stats_delivery_count": "1", "stats_event_log_dropped": "0",
        "metric_closed_inflated_voxel_volume": "true",
        "metric_epsilon_present": "true", "metric_epsilon": "0.4",
        "metric_epsilon_unchanged": "true",
        "metric_inflation_axes_present": "true",
        "metric_inflation_x": "0.2", "metric_inflation_y": "0.2",
        "metric_inflation_z": "0.35",
        "metric_inflation_provenance": "sdfmap-layered-effective-backing",
        "metric_observed_obstacle_containment_proven": "false",
        "metric_vehicle_tracking_budget_proven": "false",
        "metric_physical_containment_proven": "false",
        "metric_support_complete": "true",
    })
    event = {field: "not_applicable" for field in fields}
    event.update({
        "schema_version": SCHEMA_VERSION, "record_kind": "EVENT",
        "ordinal": "0", "capability_state": "CERTIFIED_PROFILE",
        "planner_only": "true", "event_type": "COMPLETION_DELIVERED",
        "event_timestamp_ticks": "20", "event_purpose": "CURRENT",
        "event_request_id": "8589934592",
        "event_execution_generation": "4294967296",
        "event_accepted_state_demand": "25769803776",
    })
    boundary = {field: "not_applicable" for field in fields}
    boundary.update({
        "schema_version": SCHEMA_VERSION, "record_kind": "BOUNDARY",
        "ordinal": "0", "capability_state": "CERTIFIED_PROFILE",
        "planner_only": "true", "boundary_index": "0",
        "boundary_live_k_present": "false",
    })
    out = io.StringIO(newline="")
    writer = csv.DictWriter(out, fieldnames=fields, lineterminator="\n")
    writer.writeheader()
    writer.writerows((snapshot, event, boundary))
    return out.getvalue()


def self_test():
    fixture = _fixture()
    first = replay_trace(fixture)
    second = replay_trace(first)
    if first != second:
        raise ReplayError("replay output is not byte-stable")
    digest = hashlib.sha256(first.encode("utf-8")).hexdigest()
    # Reordered/dropped/duplicated/identity-mismatched records must be caught.
    rows = list(csv.reader(io.StringIO(first)))
    header = rows[0]
    def mutated(records):
        output = io.StringIO(newline="")
        writer = csv.writer(output, lineterminator="\n")
        writer.writerow(header)
        writer.writerows(records)
        return output.getvalue()

    def expect_replay_error(label, records, expected_fragment=None):
        try:
            replay_trace(mutated(records))
        except ReplayError as error:
            if expected_fragment is not None and \
                    expected_fragment not in str(error):
                raise ReplayError(
                    "%s mutation failed for an unexpected reason: %s" %
                    (label, error))
            return
        raise ReplayError("%s mutation was accepted" % label)

    # Swap the event and boundary: grouping by kind alone must not make this
    # malformed source order acceptable.
    expect_replay_error("reordered", [rows[1], rows[3], rows[2]])
    expect_replay_error("duplicated", [rows[1], rows[2], rows[2], rows[3]])
    expect_replay_error("dropped", [rows[1], rows[3]])
    identity_mismatch = [list(row) for row in rows[1:]]
    request_column = header.index("event_request_id")
    identity_mismatch[1][request_column] = "8589934593"
    expect_replay_error("identity-mismatched", identity_mismatch, "identity")
    print("offline replay self-test ok sha256=%s" % digest)


def test_offline_replay_self_test():
    """Nose/CTest entry point for the deterministic offline oracle."""
    self_test()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trace", help="canonical source-recorded CSV")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args(argv)
    try:
        if args.self_test or not args.trace:
            self_test()
            return 0
        with open(args.trace, "r", encoding="utf-8", newline="") as stream:
            canonical = replay_trace(stream.read())
        print("replay ok sha256=%s" %
              hashlib.sha256(canonical.encode("utf-8")).hexdigest())
        return 0
    except (OSError, ReplayError, csv.Error) as error:
        print("replay error: %s" % error, file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
