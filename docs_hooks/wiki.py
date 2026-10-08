"""MkDocs hook that lets the site build straight from GitHub wiki markdown.

- Builds the site navigation from the wiki's _Sidebar.md, so new wiki pages show up
  on the site as soon as they're added to the sidebar.
- Serves Home.md as the site's front page (index.html).
- Rewrites wiki-style links like [x](Path-Planning#anchor) to [x](Path-Planning.md#anchor)
  so MkDocs can resolve them. GitHub turns "+" into "-" in page filenames, so
  "ROS2-Tutorial-(C++)" maps to "ROS2-Tutorial-(C--).md".
- Copies the .js/.css files in docs_hooks/ next to the built site.
"""

import logging
import os
import re
import shutil

log = logging.getLogger("mkdocs.hooks.wiki")

_link_re = None
# Allows one level of parentheses inside the target, e.g. ROS2-Tutorial-(C++)
_sidebar_link_re = re.compile(r"\[([^\]]+)\]\(((?:[^()\s]|\([^()\s]*\))+)\)")
_sidebar_heading_re = re.compile(r"^\*\*([^*\[\]]+)\*\*$")


def _sidebar_target(target, docs_dir):
    """Map a wiki link target to a docs file, or None if no such page exists."""
    if "://" in target:
        return target
    target = target.split("#")[0]
    for name in (target, target.replace("+", "-")):
        if os.path.isfile(os.path.join(docs_dir, name + ".md")):
            return name + ".md"
    return None


def _nav_from_sidebar(path, docs_dir):
    """Turn _Sidebar.md into an MkDocs nav.

    A bold line with a link (**[Home](Home)**) is a top-level page; a bold line
    without one (**Concepts**) starts a section; list items below it are its pages.
    A page listed more than once only appears under its first listing.
    """
    nav, section, seen = [], None, set()
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            heading = _sidebar_heading_re.match(line)
            if heading:
                section = []
                nav.append({heading.group(1).strip(): section})
                continue
            link = _sidebar_link_re.search(line)
            if not link:
                continue
            title, target = link.group(1).strip(), _sidebar_target(link.group(2), docs_dir)
            if target is None:
                log.info("_Sidebar.md links to missing page %r, skipping", link.group(2))
                continue
            # MkDocs files a page under the last section that lists it, which makes the
            # earlier tab open the wrong section. Keep only the first listing.
            if target in seen:
                log.info("_Sidebar.md lists %r more than once, keeping the first", target)
                continue
            seen.add(target)
            is_item = re.match(r"^([-*+]|\d+\.)\s", line)
            if is_item and section is not None:
                section.append({title: target})
            else:
                section = None
                nav.append({title: target})
    return [entry for entry in nav if list(entry.values())[0] != []]


def on_config(config):
    sidebar = os.path.join(config["docs_dir"], "_Sidebar.md")
    if os.path.isfile(sidebar):
        config["nav"] = _nav_from_sidebar(sidebar, config["docs_dir"])
    return config


def _wiki_name(src_uri):
    return src_uri[:-3] if src_uri.endswith(".md") else None


def on_files(files, config):
    global _link_re
    names = {}
    for f in files.documentation_pages():
        name = _wiki_name(f.src_uri)
        if name is None or "/" in name:
            continue
        names[name] = f.src_uri
        names[name.replace("--", "++")] = f.src_uri
        if name == "Home":
            f.dest_uri = "index.html"
            f.url = "./" if config["use_directory_urls"] else "index.html"

    alternatives = "|".join(re.escape(n) for n in sorted(names, key=len, reverse=True))
    _link_re = (re.compile(r"\]\((%s)(#[^)\s]*)?\)" % alternatives), names)
    return files


def on_page_markdown(markdown, page, config, files):
    pattern, names = _link_re
    return pattern.sub(lambda m: "](%s%s)" % (names[m.group(1)], m.group(2) or ""), markdown)


def on_post_build(config):
    here = os.path.dirname(__file__)
    dest = os.path.join(config["site_dir"], "docs_hooks")
    os.makedirs(dest, exist_ok=True)
    for name in os.listdir(here):
        if name.endswith((".js", ".css")):
            shutil.copy(os.path.join(here, name), dest)
