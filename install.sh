# SPDX-FileCopyrightText: © 2015 Marco Frigerio
# SPDX-License-Identifier: BSD-2-Clause
#!/bin/bash

DEST_DIR=/usr/local/include/

cp -vu --parents `find iit/ -name '*.h'` $DEST_DIR
