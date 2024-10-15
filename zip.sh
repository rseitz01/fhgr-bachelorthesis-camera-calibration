#!/bin/bash
set -e

mainfolder=FHGR-Bachelorarbeit-Seitz-Raphael

rm -rf $mainfolder | true
rm -rf $mainfolder.zip | true
mkdir $mainfolder

mkdir $mainfolder/doc/expose -p
mkdir $mainfolder/doc/images/phone -p
mkdir $mainfolder/doc/images/pwm-curves -p
mkdir $mainfolder/doc/results -p
mkdir $mainfolder/doc/measurements -p
mkdir $mainfolder/doc/tex -p
mkdir $mainfolder/doc/pres -p

mkdir $mainfolder/ledrunner/housing-ledrunner -p
mkdir $mainfolder/ledrunner/hw-ledrunner -p
mkdir $mainfolder/ledrunner/sw-camera-calibration -p
mkdir $mainfolder/ledrunner/sw-camera-calibration -p
mkdir $mainfolder/ledrunner/sw-ledrunner -p

mkdir $mainfolder/src/camera-calibration/data -p
mkdir $mainfolder/src/camera-calibration/data_low_light -p
mkdir $mainfolder/src/camera-control/src -p
mkdir $mainfolder/src/eval-dynamic/measurements -p
mkdir $mainfolder/src/eval-static -p

# copy files

cp -p README.md $mainfolder
cp -p zip.sh $mainfolder
cp -p doc/expose/*Seitz_Raphael* $mainfolder/doc/expose/
cp -p doc/images/phone/aufbau-*-cut.jpg $mainfolder/doc/images/phone/
cp -p doc/images/phone/*-edit.jpg $mainfolder/doc/images/phone/
cp -p doc/images/phone/*-edit.png $mainfolder/doc/images/phone/
cp -p doc/images/phone/*-edit.png $mainfolder/doc/images/phone/
cp -p doc/images/phone/messung-frequenzen*.jpg $mainfolder/doc/images/phone/
cp -p doc/images/pwm-curves/* $mainfolder/doc/images/pwm-curves/
cp -p doc/images/blockschaltbild.png $mainfolder/doc/images/
cp -p doc/images/blurry.jpg $mainfolder/doc/images/
cp -p doc/images/pwm-test-areas.png $mainfolder/doc/images/
cp -p doc/images/pwm-test.png $mainfolder/doc/images/
cp -p doc/images/results-calibrated-frequencies.png $mainfolder/doc/images/
cp -p doc/images/systemabgrenzung.png $mainfolder/doc/images/

cp -p doc/results/* $mainfolder/doc/results/
cp -p doc/measurements/* $mainfolder/doc/measurements/

cp -p doc/tex/*.tex $mainfolder/doc/tex/
cp -p doc/tex/*.cls $mainfolder/doc/tex/
cp -p doc/tex/run.sh $mainfolder/doc/tex/
cp -p doc/tex/*.bib $mainfolder/doc/tex/

cp -p doc/pres/* $mainfolder/doc/pres/

cp -p ledrunner/housing-ledrunner/*.pdf $mainfolder/ledrunner/housing-ledrunner/
cp -p ledrunner/housing-ledrunner/*-v16.step $mainfolder/ledrunner/housing-ledrunner/
cp -p ledrunner/housing-ledrunner/housing-ledrunner.f3d $mainfolder/ledrunner/housing-ledrunner/

cp -p ledrunner/hw-ledrunner/* $mainfolder/ledrunner/hw-ledrunner/ -r
cp -p ledrunner/sw-camera-calibration/* $mainfolder/ledrunner/sw-camera-calibration/ -r
cp -p ledrunner/sw-ledrunner/* $mainfolder/ledrunner/sw-ledrunner/ -r
cp -p ledrunner/README.md $mainfolder/ledrunner/

cp -p src/README.md $mainfolder/src
cp -p src/camera-calibration/*.py $mainfolder/src/camera-calibration/
cp -p src/camera-calibration/data/* $mainfolder/src/camera-calibration/data/
cp -p src/camera-calibration/data_low_light/* $mainfolder/src/camera-calibration/data_low_light/

cp -p src/camera-control/Makefile $mainfolder/src/camera-control/
cp -p src/camera-control/compile_flags.txt $mainfolder/src/camera-control/
cp -p src/camera-control/src/* $mainfolder/src/camera-control/src/

cp -p src/eval-dynamic/README.md $mainfolder/src/eval-dynamic/
cp -p src/eval-dynamic/*.py $mainfolder/src/eval-dynamic/
cp -p src/eval-dynamic/measurements/* $mainfolder/src/eval-dynamic/measurements/

cp -p src/eval-static/*.py $mainfolder/src/eval-static/

zip $mainfolder.zip $mainfolder -r

echo Done.

