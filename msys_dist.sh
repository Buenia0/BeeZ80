#!/bin/bash

if [[ ! -f "libbeez80.a" ]]; then
	echo "Run this script from the directory where you built the BeeZ80 engine."
	exit 1
fi


if [[ -f "beez80-tests.exe" ]]; then
	cp -r ../tests .
fi