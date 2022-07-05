#!/bin/bash

if [[ ! -f "BeeZ80/libbeez80.a" ]]; then
	echo "Run this script from the directory where you built the BeeZ80 engine."
	exit 1
fi

mkdir -p dist/

if [[ -f "beez80-tests.exe" ]]; then
	for lib in $(ldd beez80-tests.exe | grep mingw | sed "s/.*=> //" | sed "s/(.*)//"); do
		cp "${lib}" dist
	done
	cp beez80-tests.exe dist
	cp -r ../tests dist
fi