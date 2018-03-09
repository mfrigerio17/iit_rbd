#!/bin/bash

DEST_DIR=/usr/local/include/

cp -vu --parents `find iit/ -name '*.h'` $DEST_DIR
