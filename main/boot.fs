( Things loaded at boot )
: test 0 100 for dup . 1+ next drop cr ;

: docreate   r> ;
: create   : [ ' docreate ] literal , [ ' [ , ] overt ;
: does>   here last @ name> cell+ ! [ ' dolist @ ] literal , [ ' r> ] literal , ] ;

