#!/bin/bash

echo "==> Downloading source tar"
echo ""

wget -O apriltag_sources.tgz https://april.eecs.umich.edu/media/apriltag/apriltag-2015-03-18.tgz

echo "==> Extracting sources"

tar -xzf apriltag_sources.tgz
mv apriltag-2015-03-18 src

echo "==> Done"
