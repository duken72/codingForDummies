#!/bin/awk

BEGIN { print "Header line" }

/pattern/ {action}
/produce/ {print $2}
$3>1 {print $1}

END { print "Ending line" }
