( Things loaded at boot )
: test 0 100 for dup . 1+ next drop cr ;

64 ' create >name 1+ c!  ( hide create )
' dolist @ constant op_dolist
: n,   compile dolit , ;
: [']   ' n, ; immediate
: docreate   r> ;
: create   : compile docreate [compile] [ overt ;
: hookupdoes   last @ name> cell+ ! ;
: does>   'eval @ ['] $interpret = if
      here hookupdoes
   else
      here 4 cells + n, compile hookupdoes compile exit
   then
   op_dolist , compile r> [compile] ]
; immediate

: SEE
   ' BEGIN CELL+ DUP @ DUP IF >NAME THEN ?DUP
   IF SPACE .ID ELSE DUP @ U. THEN DUP @ ['] EXIT =
   UNTIL DROP ;
