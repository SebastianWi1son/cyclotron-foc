#!/usr/bin/env python3
"""
check_docs.py — 把「文档规矩」变成机器门禁

规则的唯一来源是 `docs/README.md`（§6 六条硬规则 + §7「机器能判的，不要写成文字」）。
本脚本只负责把其中【机器判得了】的那几条跑一遍 —— 不判重要性，只判可判性。

用法：
    scripts/check_docs.py            # 检查；有违规 → 退出码 1
    scripts/check_docs.py -v         # 同时打印每个受管文件的判定
    scripts/check_docs.py --list     # 只列出受管文件与它们的类
    scripts/check_docs.py --why      # 打印规则清单与各自的依据

设计说明（为什么长这样）：
  · 零依赖 —— 不用 PyYAML。CI 里不需要 pip install，跑得也快。
  · 行号规则（R3）只作用于 fact/status/work，**log 类豁免** ——
    A 类日志「只增不改」，里面的行号是【当时那一行的快照】，是正确记录，不是错误。
    硬要它无行号，等于要求历史记录不许带时间戳。
  · 存量违规走 `scripts/doc_lint_baseline.txt`（债务清单）而不是豁免规则 ——
    数字是明示的、可数的、只减不增的；新增违规立刻变红。
"""

import argparse
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

# 不受管的目录：
#   trash/     待裁决，即将整目录删除，给它补元数据没有意义
#   reference/ 供应商代码（不入 git）
#   legacy/    冻结的旧实现，只读参考（AGENTS.md：不许改）
#   build/ 等  构建产物
SKIP_DIRS = {"trash", "reference", "legacy", "build", "node_modules", ".git", ".venv"}

CLASSES = {"log", "fact", "status", "work"}
GENERATED = {"true", "false"}

SRC_EXT = r"(?:cpp|hpp|hh|h|cc|c|py|md|txt|cmake|yml|yaml|json|sh)"
# 行号引用： foo/bar.cpp:25   或   foo/bar.hpp:25-51
LINE_REF = re.compile(
    rf"[\w./-]+\.{SRC_EXT}\s*:\s*\d+(?:\s*-\s*\d+)?"
)
# 中文写法的行号：「第 68 行」—— 同一种锚定，同样会烂，不能漏
LINE_REF_CN = re.compile(r"第\s*\d+\s*行")
LINE_RES = (LINE_REF, LINE_REF_CN)
# markdown 本地链接
MD_LINK = re.compile(r"\[[^\]]*\]\(\s*<?([^)>\s]+)>?(?:\s+\"[^\"]*\")?\s*\)")
# 反引号里的内容
BACKTICK = re.compile(r"`([^`\n]+)`")
# 反引号里「像仓库内文件」的：带路径的（inc/odometry.hpp）
PATHY = re.compile(rf"^[\w.-]+(?:/[\w.-]+)+\.{SRC_EXT}$")
# 或者不带路径的裸文件名（speed_limiter.hpp）——
# 只认【带已知扩展名】的，避开 "每个组件的 DESIGN.md" 这类泛指以外的噪声
BARE = re.compile(rf"^[\w.-]+\.{SRC_EXT}$")
FENCE = re.compile(r"^\s*(?:```|~~~)")
# 显式标注为「计划中」的行 —— 这些行里的文件名是路线图，不是在声称仓库里有。
# 例：结构树里写 `⬜ mat3.hpp（批次 4）`。
PLANNED_MARKS = ("⬜",)

# 代码块里不带反引号的文件名（目录树 / 架构图 / 构建命令）
BARE_ANY = re.compile(rf"[\w./-]+\.{SRC_EXT}\b")


# ───────────────────────── 基础工具 ─────────────────────────

def nested_repos(root: Path, skip_under_skip_dirs: bool = True):
    """顶层以下的【嵌套 git 仓库】—— 那是另一个项目，不属于本仓。

    为什么需要：一个项目里同时住着“项目文档”和“独立发布的子库”是常见形态
    （子库有自己的仓、自己的 CI）。

    判据用 `.git` 目录本身，不维护目录名白名单 —— 白名单会跟实际的仓分布各说各话。
    """
    out = []
    for p in root.rglob(".git"):
        if not p.is_dir() or p.parent == root:
            continue
        rel = p.parent.relative_to(root)
        if skip_under_skip_dirs and any(part in SKIP_DIRS for part in rel.parts):
            continue  # reference/ 这类已在 SKIP_DIRS（供应商代码），不用它做存在性判据
        out.append(rel)
    return out


def is_under(rel: Path, prefixes) -> bool:
    return any(rel == pre or pre in rel.parents for pre in prefixes)


def _git_ls(root: Path):
    # -z + core.quotePath=false：不然 git 会把中文/空格文件名转义成
    # "docs/log/\345\244\215\347\233\230.md" 这种形式，导致全部误判“不存在”。
    out = subprocess.run(
        ["git", "-c", "core.quotePath=false", "-C", str(root),
         "ls-files", "-z", "-co", "--exclude-standard"],
        capture_output=True, text=True, check=True).stdout
    return {p for p in out.split("\0") if p}


def tracked_files(root: Path):
    """仓库里【应当存在】的文件（相对路径集合）。

    为什么不用 rglob：`reference/` 这类目录在本地存在但不入库，
    拿它做存在性判断会造成【本地过、CI 挂】的假绿。

    为什么带 `--others --exclude-standard`：只看 `--cached` 的话，
    刚建好还没 `git add` 的新文件会被判成“不存在”，写文档时很难用。
    加上未跟踪但未被 ignore 的文件，既修了这个坑，又仍然把 `reference/` 挡在外面。

    为什么要并上【嵌套子仓】的文件：项目级文档里画子库的目录树，是合法且常见的
    （“foc 库长这样”）。子仓是独立仓库，它的文件不在本仓的 `git ls-files` 里
    —— 不并进来的话，如实描述子库布局反倒全被误判成“文件不存在”。
    子仓自己的文档由【它自己的门禁】管（它有自己的仓与 CI）。
    """
    try:
        files = _git_ls(root)
    except Exception:  # 非 git 环境（打包分发等）→ 退回扫盘
        return {p.relative_to(root).as_posix() for p in root.rglob("*") if p.is_file()}
    for sub in nested_repos(root):
        try:
            files |= {(sub / f).as_posix() for f in _git_ls(root / sub)}
        except Exception:
            pass
    return files


def is_external(root: Path, tok: str) -> bool:
    """这个路径是否落在【被 gitignore 的目录】下 —— 那是外部引用，不是对本仓的声称。

    为什么需要：`reference/` 这类目录按 `.gitignore` 就不入库，文档里引用它是
    合法的（叫人去本地那个目录看源码）。不区分的话，这类引用永远过不了门禁。

    判据交给 git 自己（`check-ignore`），不再维护一份“外部路径前缀”白名单
    —— 那份名单会与 `.gitignore` 各说各话。
    """
    if tok in _IGNORE_CACHE:
        return _IGNORE_CACHE[tok]
    try:
        r = subprocess.run(["git", "-C", str(root), "check-ignore", "-q", tok],
                           capture_output=True)
        ok = r.returncode == 0
    except Exception:
        ok = False
    _IGNORE_CACHE[tok] = ok
    return ok


_IGNORE_CACHE = {}


def collect(root: Path):
    """返回 (受管文件列表, 跳过的文件数)"""
    managed, skipped = [], 0
    subs = nested_repos(root)
    for p in sorted(root.rglob("*.md")):
        rel = p.relative_to(root)
        if any(part in SKIP_DIRS for part in rel.parts) or is_under(rel, subs):
            skipped += 1
            continue
        managed.append(p)
    return managed, skipped


def parse_front_matter(text: str):
    """极简 front-matter 解析（只要有 key: value，不引入 YAML 依赖）"""
    lines = text.splitlines()
    if not lines or lines[0].strip() != "---":
        return None, text
    meta = {}
    for i, ln in enumerate(lines[1:], start=1):
        if ln.strip() == "---":
            return meta, "\n".join(lines[i + 1:])
        key, sep, val = ln.partition(":")
        if sep:
            # 去掉行内注释（YAML 允许 `false   # 说明`）。
            # ⚠ foucault/docs/STATUS.md 就是这么写的，曾经被当成非法值。
            val = re.sub(r"\s+#.*$", "", val)
            meta[key.strip()] = val.strip().strip("\"'")
    return None, text  # 有开头没结尾 → 视为没有


def split_fences(text: str):
    """返回 (代码块外的正文, 代码块内的内容)。

    两个用途不同：
      · 正文（outside）—— 行号规则只查它；引用旧名（改名记录 / 漂移清单）多半在正文里，
        那些是【正确的历史记录】，查了就是误报。
      · 代码块（fenced）—— 目录树 / 架构图多半在这里。它是在【声称仓库里有哪些文件】，
        所以裸文件名也查。
    """
    outside, inside, buf, is_in = [], [], [], False
    for ln in text.splitlines():
        if FENCE.match(ln):
            (inside if is_in else outside).extend(buf)
            buf = []
            is_in = not is_in
            continue
        buf.append(ln)
    (inside if is_in else outside).extend(buf)
    return "\n".join(outside), "\n".join(inside)


def tokens(text: str, allow_bare: bool):
    """取出反引号里「像仓库内文件」的 token。
    allow_bare=False 时只认带路径的（inc/odometry.hpp）；
    allow_bare=True  时也认裸文件名（drive_diff.hpp）。
    """
    for m in BACKTICK.finditer(text):
        tok = m.group(1).strip()
        if "*" in tok or " " in tok or tok.startswith(("http", "-", "~", "/")):
            continue
        if PATHY.match(tok) or (allow_bare and BARE.match(tok)):
            yield tok


def drop_planned(text: str) -> str:
    """去掉显式标注为「计划中」的行（含 ⬜ 的行）——
    那些行里的文件名是路线图，不是在声称仓库里已经有。"""
    return "\n".join(ln for ln in text.splitlines()
                     if not any(mk in ln for mk in PLANNED_MARKS))


TREE_CHARS = "├└│─┌┐┘┴┬┤"
# 代码标点：一行里出现这些，就说明它是在【贴代码】而不是【列文件】。
# 只用 ASCII —— 中文标点（，；（））在文件清单的注释里很常见，不能当代码。
CODE_PUNCT = ";{}(),=+"


def _is_layout_line(ln: str, m) -> bool:
    """这一行是在【声称仓库布局】，还是只是在【贴一段代码】？

    为什么需要：代码块里也会出现像文件名的东西 —— 最典型的是 C++ 的成员访问
    `abc_ac.c`（结构体 `abc_ac` 的字段 `c`），它长得和 `.c` 源文件一模一样。
    不区分的话，“贴代码”会被当成“声称仓库里有个 files”。

    判据（宁松勿严 —— 漏一个不如误报一堆）：
      · 行里有目录树字符（├ └ │ ─）          → 目录树
      · 行首（去空白后）是 - * # |           → 文件清单
      · 这个 token 是行内第一个非空白 token，且后面没有代码标点
        （`abc_ac.c, abc_ac.b` 后面有逗号；`abc_ac.c + center };` 后面有 `+ } ;`）
        —— 那两种都是代码里的成员访问 / 表达式续行，不是文件清单
    """
    if any(c in ln for c in TREE_CHARS):
        return True
    if ln.lstrip()[:1] in "-*#|":
        return True
    head = ln[:m.start()].rstrip()
    if head and not all(c in TREE_CHARS for c in head.lstrip()):
        return False  # 前面还有别的代码 → 不是清单
    return not any(c in ln[m.end():] for c in CODE_PUNCT)


def bare_names(fenced: str):
    """代码块里【不带反引号】的文件名（目录树 / 文件清单）。

    去掉 `//` 之后的内容 —— 那是注释里的引用（多半指外部库），不是对仓库布局的声称。
    并且只取【处于布局位置】的（见 `_is_layout_line`）—— 代码块里的 C++ 代码不算。
    """
    for ln in fenced.splitlines():
        for m in BARE_ANY.finditer(ln.split("//", 1)[0]):
            if _is_layout_line(ln, m):
                yield m.group(0)


# ───────────────────────── 规则 ─────────────────────────

def check(paths, root):
    """跑全部规则。返回 (errors, warnings)，元素为 (规则代号, 相对路径, 说明)"""
    errors, warnings = [], []
    texts = {}
    for p in paths:
        texts[p] = p.read_text(encoding="utf-8", errors="replace")

    # 以 git 跟踪的文件为「存在」的准绳
    tracked = tracked_files(root)
    basenames = {Path(t).name for t in tracked}

    # 先收集每份文件的名字，供 R5（孤儿）用
    names = {p: p.relative_to(root).as_posix() for p in paths}

    for p in paths:
        rel = names[p]
        text = texts[p]
        meta, body_raw = parse_front_matter(text)
        body, fenced = split_fences(body_raw)
        in_log = "log" in p.relative_to(root).parts
        in_docs = "docs" in p.relative_to(root).parts

        # ── R1 文件头 ──
        # 注意：class 非法时【不 continue】—— 否则第一个错会把后面所有错遮住。
        #       把 cls 置 None（= “类未知”），再按“未知类”把与类无关的规则跑完。
        cls = None
        if meta is None:
            errors.append(("R1", rel, "缺少 front-matter（§3）"))
        else:
            cls = meta.get("class")
            if cls not in CLASSES:
                errors.append(("R1", rel,
                               f"class 缺失或非法：{cls!r}（只准 {'/'.join(sorted(CLASSES))}）"))
                cls = None
            if meta.get("generated") not in GENERATED:
                errors.append(("R1", rel,
                               f"generated 缺失或非法：{meta.get('generated')!r}（只准 true/false）"))

        # ── R2 类与位置相符（§5 目录地图）—— 需要 class 合法才判得了 ──
        if cls is not None:
            if cls == "log" and not in_log:
                errors.append(("R2", rel, "class: log 必须住在 log/ 目录下"))
            if cls in ("fact", "status") and in_log:
                errors.append(("R2", rel, f"class: {cls} 不许住在 log/（log/ 只放只增不改的日志）"))
            if cls == "work":
                acc = meta.get("accepted")
                if acc not in GENERATED:
                    warnings.append(("R2", rel, "class: work 缺少 accepted: true/false（验收了吗？）"))
                elif acc == "true" and in_docs:
                    errors.append(("R2", rel, "施工单已 accepted: true，必须移出 docs/（§7）"))

        # ── R3 禁止行号引用（§6④）—— 只有 log 豁免；类未知时照样查 ──
        if cls != "log":
            for pat in LINE_RES:
                for m in pat.finditer(body):
                    errors.append(("R3", rel, f"出现行号引用 `{m.group(0)}`（§6④ 禁止行号）"))

        # ── R4 本地链接必须存在（§7）──
        for m in MD_LINK.finditer(body):
            target = m.group(1).strip()
            if target.startswith(("http://", "https://", "mailto:", "#", "data:")):
                continue
            target = target.split("#", 1)[0]
            if not target:
                continue
            if not (p.parent / target).exists():
                errors.append(("R4", rel, f"链接指向不存在的路径：`{target}`"))

        # ── R5 引用的文件必须存在（§7）。log 类豁免（只增不改，里面的旧路径是当时的快照）──
        # 查两个地方，因为「引用旧名」和「声称布局」长得一样但性质相反：
        #   · 正文里【带路径】的（inc/speed_limiter.hpp）→ 在声称文件在哪 → 查
        #   · 代码块里的文件名（目录树 / 架构图不带反引号）→ 在声称仓库布局 → 查
        #   但正文里的裸文件名不查 —— 它多半出现在改名记录 / 漂移清单里，是正确的历史记录。
        if cls != "log":
            cands = set(tokens(drop_planned(body), allow_bare=False)) \
                | set(bare_names(drop_planned(fenced)))
            for tok in sorted(cands):
                if tok in tracked or Path(tok).name in basenames:
                    continue
                if (p.parent.relative_to(root) / tok).as_posix() in tracked:  # 相对这份文档的写法
                    continue
                if is_external(root, tok):  # 落在 gitignore 下 → 外部引用，不是在声称本仓有
                    continue
                errors.append(("R5", rel, f"引用了不存在的文件：`{tok}`"))

    # ── R6 fact 文档不许是孤儿（§4 每条事实只有一个家）──
    for p in paths:
        meta, _ = parse_front_matter(texts[p])
        if not meta or meta.get("class") != "fact":
            continue
        rel = names[p]
        base = p.name
        refs = 0
        for q in paths:
            if q == p:
                continue
            if rel in texts[q] or base in texts[q]:
                refs += 1
        if refs == 0:
            errors.append(("R6", rel, "fact 类文档没有任何地方引用它 —— 等于没写（§4）"))

        # ── 附加提示：status 还是手写的（§6③ 目标是脚本生成）──
        if meta.get("class") == "status" and meta.get("generated") == "false":
            warnings.append(("提示", rel, "class: status 但 generated: false —— 目标是由脚本生成（§6③）"))

    return errors, warnings


# ───────────────────────── 债务清单 ─────────────────────────

BASELINE = ROOT / "scripts" / "doc_lint_baseline.txt"


def load_baseline():
    """已知违规（历史债务）。格式： <规则>  <相对路径>  <允许条数>"""
    if not BASELINE.exists():
        return {}
    out = {}
    for ln in BASELINE.read_text(encoding="utf-8").splitlines():
        ln = ln.split("#", 1)[0].strip()
        if not ln:
            continue
        parts = ln.split()
        if len(parts) != 3:
            continue
        rule, path, n = parts
        out[(rule, path)] = int(n)
    return out


def apply_baseline(errors):
    """按 (规则, 路径) 分组，超出 baseline 的才算错，少于 baseline 的提示更新"""
    base = load_baseline()
    counts = {}
    for rule, path, msg in errors:
        counts.setdefault((rule, path), []).append(msg)

    remaining, shrunk, fixed = [], [], []
    for key, msgs in counts.items():
        allowed = base.get(key, 0)
        if len(msgs) > allowed:
            for extra in msgs[allowed:]:
                remaining.append((key[0], key[1], extra))
        if len(msgs) < allowed:
            shrunk.append((key[0], key[1], allowed, len(msgs)))
    for key, allowed in base.items():
        if key not in counts:
            fixed.append((key[0], key[1], allowed))
    return remaining, shrunk, fixed


# ───────────────────────── 主流程 ─────────────────────────

WHY = """\
规则来源：docs/README.md（§6 六条硬规则 / §7「机器能判的，不要写成文字」）

  R1  每份受管文档必须有 front-matter，class ∈ {log,fact,status,work}，
      generated ∈ {true,false}                                      §3 文件头模板
  R2  类必须和位置相符：log → 只能住 log/；fact/status → 不许住 log/；
      work → accepted: true 之后必须移出 docs/                       §5 目录地图 / §7
  R3  fact / status / work 正文禁止「文件:行号」；log 豁免            §6④
  R4  markdown 本地链接的目标必须存在                                §7
  R5  反引号里的仓库内路径必须存在                                   §7
  R6  fact 类文档不许是孤儿（必须被别处引用）                        §4 事实归属表

不受管：trash/（待裁决）· reference/（供应商）· legacy/（冻结参考）· build/

存量违规记在 scripts/doc_lint_baseline.txt（豁免清单）：只该装【有理由的例外】，
不该装【还没修的债】—— 债要还，例外要写清理由。
"""


def main():
    ap = argparse.ArgumentParser(description="文档门禁：把 docs/README.md 的规矩跑一遍")
    ap.add_argument("-v", "--verbose", action="store_true", help="打印每个受管文件的判定")
    ap.add_argument("--list", action="store_true", help="只列出受管文件与它们的类")
    ap.add_argument("--why", action="store_true", help="打印规则清单与依据")
    args = ap.parse_args()

    if args.why:
        print(WHY)
        return 0

    # 防呆：脚本必须住在 <仓库>/scripts/ 下（ROOT 由脚本位置反推）。
    # 拷到 /tmp 之类的地方跑，会让 ROOT 变成 `/`，然后 rglob 一路爬进 /proc 崩掉。
    if ROOT == ROOT.parent:
        print(f"❌ 脚本位置不对：ROOT 解析成了文件系统根 `{ROOT}`。\n"
              f"   它必须放在 <仓库>/scripts/check_docs.py —— ROOT = 脚本所在目录的上一级。")
        return 2

    paths, skipped = collect(ROOT)

    if args.list:
        print(f"受管文档 {len(paths)} 个（跳过 {skipped} 个：trash/ reference/ legacy/ build/）\n")
        print(f"{'类':8s} {'生成':6s} 路径")
        for p in paths:
            meta, _ = parse_front_matter(p.read_text(encoding="utf-8", errors="replace"))
            meta = meta or {}
            print(f"{meta.get('class','?'):8s} {meta.get('generated','?'):6s} "
                  f"{p.relative_to(ROOT).as_posix()}")
        return 0

    errors, warnings = check(paths, ROOT)
    remaining, shrunk, fixed = apply_baseline(errors)

    print(f"文档门禁  {ROOT.name}")
    print(f"受管 {len(paths)} 个 md（跳过 {skipped} 个：trash/ reference/ legacy/ build/）")
    if load_baseline():
        n_debt = sum(load_baseline().values())
        print(f"豁免清单 {len(load_baseline())} 条 / 共 {n_debt} 处（有理由的例外，不是未修的债）")
    print()

    if args.verbose:
        for p in paths:
            meta, _ = parse_front_matter(p.read_text(encoding="utf-8", errors="replace"))
            meta = meta or {}
            print(f"  · {p.relative_to(ROOT).as_posix():52s} {meta.get('class','?'):7s} "
                  f"generated={meta.get('generated','?')}")
        print()

    for rule, path, msg in remaining:
        print(f"❌ {rule}  {path}\n      {msg}")
    for rule, path, msg in warnings:
        print(f"⚠️  {rule}  {path}\n      {msg}")

    if shrunk:
        print()
        for rule, path, was, now in shrunk:
            print(f"ℹ️  豁免减少：{rule} {path} —— baseline 写 {was}，实际 {now}。"
                  f"请把 scripts/doc_lint_baseline.txt 改成 {now}。")
    if fixed:
        print()
        for rule, path, was in fixed:
            print(f"ℹ️  豁免已不需要：{rule} {path}（原 {was} 处）—— 从 baseline 里删掉这一行。")

    print()
    if remaining:
        print(f"❌ {len(remaining)} 处违规 —— 门禁不通过")
        return 1
    print(f"✅ 全部通过（{len(warnings)} 条提示）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
