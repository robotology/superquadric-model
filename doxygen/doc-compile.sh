#!/bin/bash

DESTINATIONXML=generated-from-xml

# clean-up
rm -Rf doc
rm -Rf $DESTINATIONXML

# generate doxy from xml
mkdir $DESTINATIONXML
list=`find .. -iname *.xml | xargs`
for i in $list
do
   filename=`basename $i`
   doxyfile=${filename%%.*}
   xsltproc --output $DESTINATIONXML/$doxyfile.dox $YARP_ROOT/scripts/yarp-module.xsl $i
done

doxygen ./generate.txt
