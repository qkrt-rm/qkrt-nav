"""MkDocs hook that lets the site build straight from GitHub wiki markdown.

- Serves Home.md as the site's front page (index.html).
- Rewrites wiki-style links like [x](Path-Planning#anchor) to [x](Path-Planning.md#anchor)
  so MkDocs can resolve them. GitHub turns "+" into "-" in page filenames, so
  "ROS2-Tutorial-(C++)" maps to "ROS2-Tutorial-(C--).md".
- Copies mathjax.js next to the built site.
"""

import os
import re
import shutil

_link_re = None


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
    shutil.copy(os.path.join(here, "mathjax.js"), dest)
