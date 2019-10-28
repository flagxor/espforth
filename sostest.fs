: on 0 13 pin ;
: off 1 13 pin ;
: dot on 100 ms off 100 ms ;
: dash on 300 ms off 100 ms ;
: gap 300 ms ;
: sos dot dot dot gap dash dash dash gap dot dot dot gap ;
: test begin sos gap again ;
test

