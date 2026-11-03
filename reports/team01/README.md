# TEAM 01 independent QA reports

This directory is maintained by the **verification role**; it does not own vehicle controller implementation.

- [2026-11-03 target af287f8 baseline](2026-11-03-af287f8-baseline.md)
- [Independent Actions workflow](../../.github/workflows/team01-qa-validation.yml)
- [All GitHub Actions runs](https://github.com/nahee2345/urrc_2_f1/actions/workflows/team01-qa-validation.yml)

**Verdicts:** PASS only with real independent evidence; FAIL on unit regressions, sensor failures, invalid start, offtrack, lap timeout, or SLAM integrity failures where tested; NOT TESTED means no completed physical test. QA artifacts are preserved per run under the Actions tab.

Future changes to driving/SLAM code must be made by the implementation chat, not in QA reports.
