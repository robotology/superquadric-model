
$DESTINATIONXML="generated-from-xml"

if (test-path doc) {
    rm doc -Recurse
}

if (test-path $DESTINATIONXML) {
    rm $DESTINATIONXML -Recurse
}

if (!(test-path $DESTINATIONXML)) {
    mkdir $DESTINATIONXML
}

ls -Path .. -Filter *.xml -Recurse | `
foreach-object {xsltproc --output $DESTINATIONXML/$_.dox `
$env:YARP_ROOT\scripts\yarp-module.xsl $_.fullname}

doxygen ./generate.txt
