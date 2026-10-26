#!/usr/bin/env python3
#
# Moves the pin of a third-party library (TPL) that pe downloads through pe_get_tpl().
#
# Copyright (C) 2026 Raphael Münster
# Adapted from FEAT3, build_system/bump_tpl_pin.py, Copyright (C) the FEAT group, GPL-3.
#
# This file is part of pe. pe is free software: you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software Foundation,
# either version 3 of the License, or (at your option) any later version. See
# <http://www.gnu.org/licenses/>.

"""
Moves the pin of a third-party library downloaded by pe_get_tpl() (cmake/PEGetTPL.cmake).

Every pe_get_tpl() call in CMakeLists.txt pins an archive by its URLs (upstream first, then the
copy in the pe-third-party mirror) and its hash. This script

  1. downloads the new upstream archive and computes its hash (with the algorithm of the
     current pin, SHA256 for all pins today),
  2. downloads the mirror copy, if a mirror URL is given, and checks that it is identical,
  3. stores the archive in the TPL cache under the name pe_get_tpl() looks for,
  4. rewrites VERSION, URLS and URL_HASH of the pe_get_tpl() call.

  python3 tools/bump_tpl_pin.py nlohmann_json 3.12.1 \\
      https://github.com/nlohmann/json/releases/download/v3.12.1/json.tar.xz \\
      --mirror https://github.com/tudo-math-ls3/pe-third-party/releases/download/nlohmann-json-3.12.1/json.tar.xz

The hash this script computes only says what the server delivered today. Before committing,
verify the archive against an independent source (a distribution's source package, a published
checksum or signature) and note it in doc/technical-notes/build-with-cmake.md. Only Python 3 is
needed.
"""

import argparse
import hashlib
import os
import re
import sys
import urllib.error
import urllib.request

PE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def error(*msg):
    print("ERROR:", *msg, file=sys.stderr)
    sys.exit(1)


def find_block(text, package):
    """returns (start, end) of the pe_get_tpl(...) call with the given PACKAGE_NAME"""
    for m in re.finditer(r"pe_get_tpl\s*\(", text):
        depth, i, comment = 1, m.end(), False
        while depth > 0 and i < len(text):
            c = text[i]
            if c == "#":
                comment = True
            elif c == "\n":
                comment = False
            elif not comment:
                depth += {"(": 1, ")": -1}.get(c, 0)
            i += 1
        block = text[m.start():i]
        if re.search(r"PACKAGE_NAME\s+%s\s" % re.escape(package), block):
            return m.start(), i
    return None


def download(url, algo):
    """returns (hex digest, data) of the file at url"""
    req = urllib.request.Request(url, headers={"User-Agent": "pe-bump-tpl-pin"})
    h = hashlib.new(algo.lower())
    data = bytearray()
    with urllib.request.urlopen(req, timeout=300) as r:
        while True:
            chunk = r.read(1 << 20)
            if not chunk:
                break
            h.update(chunk)
            data += chunk
    return h.hexdigest(), bytes(data)


def main():
    ap = argparse.ArgumentParser(description="Move the pin of a pe third-party library.")
    ap.add_argument("package", help="PACKAGE_NAME of the pe_get_tpl() call, e.g. Eigen3")
    ap.add_argument("version", help="version of the new archive, e.g. 3.4.1")
    ap.add_argument("url", help="upstream URL of the new archive")
    ap.add_argument("--mirror", help="URL of the copy in the pe-third-party mirror (without it, "
                                     "the pin has only the upstream URL)")
    ap.add_argument("--file", default=os.path.join(PE_ROOT, "CMakeLists.txt"),
                    help="CMake file with the pe_get_tpl() calls (default: CMakeLists.txt)")
    ap.add_argument("--cache", default=os.path.join(PE_ROOT, "thirdparty", "cache"),
                    help="TPL archive cache, empty to disable (default: thirdparty/cache)")
    ap.add_argument("--dry-run", action="store_true", help="only report what would change")
    args = ap.parse_args()

    with open(args.file, encoding="utf-8") as f:
        text = f.read()
    span = find_block(text, args.package)
    if span is None:
        error("no pe_get_tpl(PACKAGE_NAME %s ...) in %s" % (args.package, args.file))
    block = text[span[0]:span[1]]
    m_version = re.search(r"^([ \t]*VERSION[ \t]+)(\S+)", block, re.M)
    m_urls = re.search(r"^([ \t]*URLS[ \t]+)(.*?)\n(?=[ \t]*(?:[A-Z][A-Z_]*\b|\)))",
                       block, re.M | re.S)
    m_hash = re.search(r"^([ \t]*URL_HASH[ \t]+)([A-Za-z0-9_]+)=([0-9a-fA-F]+)", block, re.M)
    if not m_version or not m_urls or not m_hash:
        error("cannot find VERSION, URLS and URL_HASH in the pe_get_tpl() call of", args.package)
    old_urls = m_urls.group(2).split()
    algo, old_hash = m_hash.group(2).upper(), m_hash.group(3).lower()

    print("package : %s" % args.package)
    print("old pin : %s %s\n          %s\n          %s=%s"
          % (args.package, m_version.group(2), "\n          ".join(old_urls), algo, old_hash))
    try:
        new_hash, data = download(args.url, algo)
    except (urllib.error.URLError, OSError) as e:
        error("cannot download %s: %s" % (args.url, e))
    print("new pin : %s %s\n          %s (%d bytes)\n          %s=%s"
          % (args.package, args.version, args.url, len(data), algo, new_hash))

    new_urls = [args.url]
    mirror_missing = False
    if args.mirror:
        new_urls.append(args.mirror)
        try:
            mirror_hash, _ = download(args.mirror, algo)
            if mirror_hash != new_hash:
                error("the mirror copy %s has %s=%s, not %s" % (args.mirror, algo, mirror_hash,
                                                                new_hash))
            print("mirror  : %s, identical" % args.mirror)
        except (urllib.error.URLError, OSError) as e:
            mirror_missing = True
            print("mirror  : %s not available yet (%s)" % (args.mirror, e))
    else:
        print("mirror  : none given, the pin will only have the upstream URL")

    if args.dry_run:
        print("dry run, %s not changed" % args.file)
        return

    if args.cache:
        os.makedirs(args.cache, exist_ok=True)
        cached = os.path.join(args.cache, "%s-%s-%s" % (args.package, args.version,
                                                         os.path.basename(args.url)))
        with open(cached, "wb") as f:
            f.write(data)
        print("cached  : %s" % cached)

    indent = " " * len(m_urls.group(1))
    urls = ("\n" + indent).join(new_urls)
    new_block = (block[:m_version.start(2)] + args.version + block[m_version.end(2):m_urls.start(2)]
                 + urls + block[m_urls.end(2):])
    m_hash = re.search(r"^([ \t]*URL_HASH[ \t]+)([A-Za-z0-9_]+)=([0-9a-fA-F]+)", new_block, re.M)
    new_block = new_block[:m_hash.start(3)] + new_hash + new_block[m_hash.end(3):]
    with open(args.file, "w", encoding="utf-8") as f:
        f.write(text[:span[0]] + new_block + text[span[1]:])
    print("updated : %s" % args.file)

    if mirror_missing:
        # .../releases/download/<tag>/<asset>; the asset is named after the uploaded file
        tag, asset = args.mirror.rstrip("/").split("/")[-2:]
        print("\nREMINDER: the mirror copy does not exist yet. Upload the unmodified archive as\n"
              "          asset %s of a NEW release %s of tudo-math-ls3/pe-third-party\n"
              "          before the pin is committed, e.g.\n"
              "          gh release create %s --repo tudo-math-ls3/pe-third-party <dir>/%s\n"
              "          Never replace or add a file in an existing release: a new archive always\n"
              "          gets a new release <package>-<version>."
              % (asset, tag, tag, asset))
    elif not args.mirror:
        print("\nREMINDER: the pin has no mirror URL. Upload the unmodified archive to a NEW\n"
              "          release <package>-<version> of tudo-math-ls3/pe-third-party (never\n"
              "          replace a file in an existing release) and run this script again\n"
              "          with --mirror.")
    print("\nnext    : verify the archive against an independent source (distribution source\n"
          "          package, published checksum or signature), note it in\n"
          "          doc/technical-notes/build-with-cmake.md, reconfigure, test and commit.")


if __name__ == "__main__":
    main()
