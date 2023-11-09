#!/usr/bin/env python3

import re

from typing import Optional


def separate_yaml_files_regex(
        concat_path: str,
        debug: Optional[bool] = False,
        ) -> list:
    """Separates concatenated param paths using regex. The regex configuration
    looks for temporary files created at launch execution, e.g. the path
    received '/tmp/tmpihtyw4sf/tmp/tmpf3kerkd5' will be separated in
    ['/tmp/tmpihtyw4sf', '/tmp/tmpf3kerkd5'].

    Args:
        concat_path (str): Concatenated param file paths.
        debug (Optional[bool], optional): Debug flag, prints path received and
            paths found. Defaults to False.

    Returns:
        list: List of param files paths.
    """

    files_paths = []
    files_paths = re.findall(r'/tmp/[^/]*', concat_path)

    if debug is True:
        print('Launch full params paths concatenated: {}'.format(concat_path))
        print('Paths found: {}'.format(files_paths))

    return files_paths
