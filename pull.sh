#!/bin/bash

set -x

git pull --recurse-submodules=yes

git fetch upstream

set +x
