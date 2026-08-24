from bag_motion_replay.qos import (
    QosSpec,
    merge_offered_qos,
    parse_offered_qos,
)

# Exactly what the 2026-08-13 bag stores for /commands/servo/position: Humble
# wrote the rmw enums as integers.
HUMBLE_RELIABLE = (
    '- history: 3\n'
    '  depth: 0\n'
    '  reliability: 1\n'
    '  durability: 2\n'
    '  deadline:\n'
    '    sec: 9223372036\n'
    '    nsec: 854775807\n'
    '  lifespan:\n'
    '    sec: 9223372036\n'
    '    nsec: 854775807\n'
    '  liveliness: 1\n'
    '  liveliness_lease_duration:\n'
    '    sec: 9223372036\n'
    '    nsec: 854775807\n'
    '  avoid_ros_namespace_conventions: false'
)

HUMBLE_BEST_EFFORT = HUMBLE_RELIABLE.replace('reliability: 1', 'reliability: 2')

JAZZY_STYLE = (
    '- history: keep_last\n'
    '  depth: 5\n'
    '  reliability: best_effort\n'
    '  durability: volatile\n'
)


def test_humble_integer_enums_are_understood():
    specs = parse_offered_qos(HUMBLE_RELIABLE)

    assert len(specs) == 1
    assert specs[0].reliability == 'reliable'
    assert specs[0].durability == 'volatile'
    assert specs[0].history == 'unknown'
    assert specs[0].depth == 0


def test_best_effort_sensor_profile_is_not_promoted_to_reliable():
    specs = parse_offered_qos(HUMBLE_BEST_EFFORT)

    assert specs[0].reliability == 'best_effort'


def test_jazzy_string_enums_are_understood():
    specs = parse_offered_qos(JAZZY_STYLE)

    assert specs == [
        QosSpec(
            history='keep_last',
            depth=5,
            reliability='best_effort',
            durability='volatile',
        )
    ]


def test_nested_duration_keys_do_not_leak_into_the_profile():
    # 'sec'/'nsec' live under deadline/lifespan and must not be read as top level.
    specs = parse_offered_qos(HUMBLE_RELIABLE)

    assert specs[0].depth == 0


def test_several_recorded_publishers_are_parsed_separately():
    specs = parse_offered_qos(HUMBLE_RELIABLE + '\n' + HUMBLE_BEST_EFFORT)

    assert len(specs) == 2
    assert [spec.reliability for spec in specs] == ['reliable', 'best_effort']


def test_merge_keeps_the_profile_that_satisfies_every_recorded_subscriber():
    merged = merge_offered_qos(
        [
            QosSpec(reliability='best_effort', durability='volatile', depth=1),
            QosSpec(reliability='reliable', durability='transient_local', depth=7),
        ]
    )

    assert merged.reliability == 'reliable'
    assert merged.durability == 'transient_local'
    assert merged.depth == 7


def test_empty_metadata_yields_a_usable_default():
    assert parse_offered_qos('') == []
    assert merge_offered_qos([]) == QosSpec()


def test_escaped_newlines_from_yaml_scalars_are_handled():
    specs = parse_offered_qos('- history: 3\\n  depth: 0\\n  reliability: 1')

    assert specs[0].reliability == 'reliable'


def test_rmw_constant_spelling_is_understood():
    specs = parse_offered_qos(
        '- reliability: RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT\n'
        '  durability: RMW_QOS_POLICY_DURABILITY_VOLATILE\n'
    )

    assert specs[0].reliability == 'best_effort'
    assert specs[0].durability == 'volatile'
