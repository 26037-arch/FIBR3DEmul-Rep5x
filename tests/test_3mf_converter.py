from __future__ import annotations

import math
import hashlib
import json
from pathlib import Path
import unittest

from tools.convert_3mf_assets import IDENTITY, ConversionError, apply_transform, compose, parse_transform


ROOT = Path(__file__).resolve().parents[1]


class ThreeMfTransformTests(unittest.TestCase):
    def test_translation_and_unit_independent_transform(self) -> None:
        m = parse_transform("1 0 0 0 1 0 0 0 1 10 20 30")
        self.assertEqual(apply_transform((1, 2, 3), m), (11, 22, 33))

    def test_component_then_build_transform_composition(self) -> None:
        component = parse_transform("1 0 0 0 1 0 0 0 1 10 0 0")
        build = parse_transform("0 1 0 -1 0 0 0 0 1 0 20 0")
        combined = compose(component, build)
        expected = apply_transform(apply_transform((2, 3, 4), component), build)
        actual = apply_transform((2, 3, 4), combined)
        for got, want in zip(actual, expected):
            self.assertAlmostEqual(got, want)

    def test_bad_transform_fails(self) -> None:
        with self.assertRaisesRegex(ConversionError, "12 finite"):
            parse_transform("1 0 0")
        with self.assertRaisesRegex(ConversionError, "12 finite"):
            parse_transform("1 0 0 0 1 0 0 0 1 0 0 nan")

    def test_converted_asset_hashes_match_audit_report(self) -> None:
        report = json.loads((ROOT / "assets" / "conversion-report.json").read_text(encoding="utf-8"))
        self.assertEqual(len(report["assets"]), 10)
        for asset in report["assets"]:
            payload = (ROOT / asset["output"]).read_bytes()
            self.assertEqual(hashlib.sha256(payload).hexdigest(), asset["output_sha256"], asset["output"])


if __name__ == "__main__":
    unittest.main()
