#!/bin/bash
echo "awk -f awkFD.awk test.txt"
awk -f awkFD.awk test.txt
echo

# Alternative
# chmod +x awkFD.awk
# Add #!/bin/awk -f to awkFD.awk
# Run with ./awkFD.awk

echo "gawk ' { print \$1 } ' test.txt"
gawk ' { print $1 } ' test.txt
echo

echo "gawk ' { print \$2 } ' test.txt"
gawk ' { print $2 } ' test.txt
echo

echo "gawk ' { print \$1, \$2 } ' test.txt"
gawk ' { print $1, $2 } ' test.txt
echo

# Find line with string, just like grep
# gawk '/string/ { print} ' input.txt
echo "gawk '/ee/ { print \$1, \$2 } ' test.txt"
gawk '/ee/ { print $1, $2 } ' test.txt
echo
