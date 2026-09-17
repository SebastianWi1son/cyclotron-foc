#!/usr/bin/env python3
"""
ci_local.py — 在本地把 GitHub Actions 的 job 逐条跑一遍（推之前先自查）

用法：
    scripts/ci_local.py                  # 跑全部 job（用【工作区当前状态】）
    scripts/ci_local.py --clean          # 用 git HEAD 全新建 clone（查"有东西没提交"）
    scripts/ci_local.py --job aggregate  # 只跑一个 job
    scripts/ci_local.py --list           # 列出有哪些 job
    scripts/ci_local.py -v               # 失败时打印完整输出

原理：解析 .github/workflows/ci.yml，对每个 job：
    · 展开 strategy.matrix（${{ matrix.xxx }} 用真实值替换）
    · 跳过 uses: 步骤（actions/checkout 等由 GitHub 提供）
    · 在**独立的临时工作区**里执行每一步的 run:（模仿 runner 的干净环境）
    · 退出码非 0 即该步失败

为什么需要它：CI 挂在私有仓库上，本地拿不到日志；而 GitHub 上一轮反馈要 1~2 分钟。
              本地跑一遍 3 秒，且能直接看到完整输出。

已知局限：
    · 不模拟 GitHub 的 runner 镜像（本机工具链版本可能与 ubuntu-24.04 不同）
    · 缺命令（如没装 pip）的步骤会显示为「跳过」而不是失败 —— 看汇总表里的 ⚠️
"""

import argparse
import itertools
import os
import pathlib
import shlex
import shutil
import subprocess
import sys
import tempfile

try:
    import yaml
except ImportError:  # pragma: no cover
    sys.exit("需要 PyYAML：pip install pyyaml")

REPO = pathlib.Path(__file__).resolve().parent.parent
WORKFLOW = REPO / ".github" / "workflows" / "ci.yml"

# 这些步骤由 GitHub 平台提供，本地跳过
SKIP_USES_PREFIX = ("actions/checkout", "actions/setup-python", "actions/cache")

# 复制工作区时排除的东西
#   注意：
#   · 必须带上 .git —— 否则依赖 git 的步骤（如 golden job 的 git diff --exit-code）会直接挂，
#     而 CI 上 actions/checkout 是带 .git 的。本项目 .git 只有 2.2M，复制开销可忽略。
#   · 排掉构建目录与 36M 的 reference/（后者本来就 gitignore）。
COPY_IGNORE = shutil.ignore_patterns(
    "build", "build-*", "cmake-build-*", "reference", "__pycache__", "*.pyc"
)


def load_workflow():
    if not WORKFLOW.exists():
        sys.exit(f"找不到 {WORKFLOW}")
    return yaml.safe_load(WORKFLOW.read_text(encoding="utf-8"))


def matrix_combos(job) -> list:
    """把 strategy.matrix 展开成若干组合（笛卡尔积）；没有 matrix 则返回 [{}]"""
    matrix = (job.get("strategy") or {}).get("matrix")
    if not matrix:
        return [{}]
    keys, values = [], []
    for k, v in matrix.items():
        if k in ("include", "exclude"):
            continue  # 本项目没用，暂不支持
        keys.append(k)
        values.append(v if isinstance(v, list) else [v])
    return [dict(zip(keys, combo)) for combo in itertools.product(*values)]


def render(script: str, combo: dict) -> str:
    """把 ${{ matrix.xxx }} 替换成真实值"""
    out = script
    for k, v in combo.items():
        out = out.replace("${{ matrix.%s }}" % k, str(v))
    return out


def make_workspace(tag: str, clean: bool, root: pathlib.Path) -> pathlib.Path:
    """建一个独立工作区（模仿 runner 的干净环境）"""
    dst = root / tag
    if clean:
        subprocess.run(["git", "clone", "-q", str(REPO), str(dst)], check=True)
    else:
        shutil.copytree(REPO, dst, ignore=COPY_IGNORE, symlinks=True)
    return dst


def run_script(script: str, cwd: pathlib.Path, verbose: bool):
    """跑一段 run: 脚本。返回 (状态, 输出行)  状态 ∈ {ok, fail, skip}"""
    env = dict(os.environ)
    env["GITHUB_WORKSPACE"] = str(cwd)
    env["CI"] = "true"
    r = subprocess.run(["bash", "-e", "-c", script], cwd=cwd, env=env,
                       capture_output=True, text=True)
    text = (r.stdout + r.stderr).rstrip()
    lines = text.splitlines() if text else []

    # 缺命令 → 记为跳过（例如本机没装 pip，而 CI 有 setup-python）
    if r.returncode == 127 or "command not found" in text:
        return "skip", lines
    return ("ok" if r.returncode == 0 else "fail"), lines


def run_job(name: str, spec: dict, root: pathlib.Path, clean: bool, verbose: bool):
    combos = matrix_combos(spec)
    results = []          # (组合标签, 步骤名, 状态)

    for idx, combo in enumerate(combos):
        tag = f"{name}_{idx}"
        if combos != [{}]:
            tag += "_" + "_".join(str(v).replace("/", "-") for v in combo.values())
        ws = make_workspace(tag, clean, root)

        for step in spec.get("steps", []):
            step_name = step.get("name") or ""
            if "uses" in step:
                if str(step["uses"]).startswith(SKIP_USES_PREFIX):
                    continue
                results.append((tag, f"{step['uses']}（非 actions/* 的，本地跳过）", "skip"))
                continue
            if "run" not in step:
                continue

            script = render(step["run"], combo)
            status, lines = run_script(script, ws, verbose)
            results.append((tag, step_name or script.splitlines()[0][:50], status))

            if status == "fail" and not verbose and lines:
                print("       失败输出（末 8 行）：")
                for ln in lines[-8:]:
                    print("         " + ln[:150])

    return results


def main():
    ap = argparse.ArgumentParser(description="本地跑 CI（解析 .github/workflows/ci.yml）")
    ap.add_argument("--job", help="只跑指定 job")
    ap.add_argument("--clean", action="store_true",
                    help="用 git HEAD 全新建 clone（查「有东西忘了提交」）")
    ap.add_argument("--list", action="store_true", help="列出所有 job")
    ap.add_argument("-v", "--verbose", action="store_true", help="失败时打印完整输出")
    args = ap.parse_args()

    wf = load_workflow()
    jobs = wf["jobs"]

    if args.list:
        print(f"{WORKFLOW.relative_to(REPO)} 里的 job：")
        for n, s in jobs.items():
            combos = matrix_combos(s)
            n_steps = sum(1 for st in s.get("steps", []) if "run" in st)
            print(f"  {n:22s} {s.get('name','')[:40]:42s} "
                  f"{len(combos)} 组合 × ~{n_steps} 步")
        return 0

    targets = {args.job: jobs[args.job]} if args.job else jobs
    if args.job and args.job not in jobs:
        sys.exit(f"没有这个 job：{args.job}（用 --list 看有哪些）")

    mode = "git HEAD 全新建 clone" if args.clean else "工作区当前状态（含未提交改动）"
    print(f"本地 CI：{len(targets)} 个 job · 工作区来源 = {mode}")
    print(f"          脚本 = {WORKFLOW.relative_to(REPO)}\n")

    root = pathlib.Path(tempfile.mkdtemp(prefix="ci_local_"))
    all_results = {}
    try:
        for name, spec in targets.items():
            print(f"══ {name}  ({spec.get('name','')})")
            res = run_job(name, spec, root, args.clean, args.verbose)
            all_results[name] = res
            for tag, step, status in res:
                icon = {"ok": "✅", "fail": "❌", "skip": "⚠️ "}[status]
                print(f"   {icon} {tag:34s} {step[:52]}")
            print()
    finally:
        shutil.rmtree(root, ignore_errors=True)

    # ── 汇总 ──
    print("═" * 62)
    print("汇总")
    print("═" * 62)
    failed = 0
    for name, res in all_results.items():
        bad = [r for r in res if r[2] == "fail"]
        skip = [r for r in res if r[2] == "skip"]
        if bad:
            mark, failed = "❌ 失败", failed + 1
        elif skip:
            mark = "⚠️  通过（有跳过）"
        else:
            mark = "✅ 通过"
        extra = ""
        if bad:
            extra = "  失败步骤: " + ", ".join(f"{s[:24]}" for _, s, _ in bad[:3])
        elif skip:
            extra = f"  跳过 {len(skip)} 步（本机缺工具，CI 上会跑）"
        print(f"  {name:22s} {mark}{extra}")

    print()
    if failed:
        print(f"❌ {failed} 个 job 失败 —— 别推，先修")
    else:
        print("✅ 全部通过 —— 可以推了")
    return failed


if __name__ == "__main__":
    sys.exit(main())
