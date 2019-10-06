decimal

: delay 100000 * for next ;
: on 0 13 pin ;
: off 1 13 pin ;
: dot on 3 delay off 3 delay ;
: dash on 9 delay off 3 delay ;
: gap 9 delay ;
: sos dot dot dot gap dash dash dash gap dot dot dot gap ;
: test begin sos gap again ;
test

