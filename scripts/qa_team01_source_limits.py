#!/usr/bin/env python3
"""QA-only: prohibit owned source/config files longer than 400 lines."""
import argparse
import json
import sys
from pathlib import Path

EXT = {".py", ".sh", ".js", ".ts", ".tsx", ".jsx", ".c", ".h",
       ".cc", ".cpp", ".hpp", ".sdf", ".xml", ".yaml", ".yml"}
SCOPES = ("src/team_01/team01_race", "scripts", ".github/workflows")
EXCLUDE = {"__pycache__", ".git", "build", "install", "log",
           "node_modules", ".venv", ".pytest_cache"}


def audit(root, limit):
    items = []
    for sub in SCOPES:
        base = root / sub
        if not base.exists():
            continue
        for path in base.rglob("*"):
            if (not path.is_file() or path.is_symlink()
                    or path.suffix.lower() not in EXT
                    or any(x in EXCLUDE for x in path.relative_to(root).parts)):
                continue
            with path.open(encoding="utf-8", errors="replace") as file:
                lines = sum(1 for _ in file)
            items.append({"path": str(path.relative_to(root)), "lines": lines,
                          "excess": max(0, lines-limit)})
    bad = sorted((x for x in items if x["excess"]), key=lambda x:-x["excess"])
    return {"status": "FAIL" if bad else "PASS",
            "max_lines": limit, "checked_files": len(items),
            "violations": bad,
            "count_rule": "physical lines including comments and blanks",
            "scope": list(SCOPES)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=str(Path(__file__).resolve().parents[1]))
    ap.add_argument("--limit", default=400, type=int)
    ap.add_argument("--report", default="ci_artifacts/source_line_limits.json")
    args = ap.parse_args()
    root = Path(args.root).resolve()
    result = audit(root, args.limit)
    dest = Path(args.report)
    if not dest.is_absolute():
        dest = root / dest
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n",
                    encoding="utf-8")
    print(f"[QA] SOURCE_SIZE={result['status']} files={result['checked_files']} limit={args.limit}")
    for item in result["violations"]:
        print(f"[QA] OVER_400 {item['path']}: {item['lines']} lines (+{item['excess']})")
    return 1 if result["violations"] else 0


if __name__ == "__main__":
    sys.exit(main())
