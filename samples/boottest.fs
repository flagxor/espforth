: bootfs   $" /spiflash/boot.fs" count ;
: sample   $" : test 0 100 for dup . 1+ next drop cr ; : on 0 13 pin ; : off 1 13 pin ; : test2 20 for off 200 ms on 200 ms next ; test test2 test" count ;
: install   bootfs w/o create-file drop dup sample rot write-file drop close-file ;
install

