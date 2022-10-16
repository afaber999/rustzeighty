;*************************************************************
;
;                 TINY BASIC FOR ZILOG Z80
;                       VERSION 2.0
;                     BY LI-CHEN WANG
;
;                  MODIFIED AND TRANSLATED
;                    TO INTEL MNEMONICS
;                     BY ROGER RAUSKOLB
;                      10 OCTOBER,1976
;
;                  MODIFIED AND TRANSLATED
;                    TO ZILOG MNEMONICS
;                      BY DOUG GABBARD
;            www.retrodepot.net
;
;                  MODIFIED AND FIXED TO WORK
;                  ON THE TEC-1F Z80 SBC USING
;                  ON BOARD BIT BANG SERIAL
;                  BY BRIAN CHIHA, JUNE 2022 
;
;           RELEASED TO THE PUBLIC
;                      10 OCTOBER,2017
;                  YEAH, 41 YEARS LATER....
;
;                         @COPYLEFT
;                   ALL WRONGS RESERVED
;
;*************************************************************
; This code is derived from the original 8080 Tiny Basic.
; It was first compiled in 8080 Mnemonics, then disassembled
; into Zilog Mnemonics.  And then checked against the original
; to ensure accuracy.  It was then partially enhanced with z80
; specific code. And once done, it was then modified to work
; with the G80-S Micro Computer. However, that portion of the
; code has been left out in order to make this code a little
; more portable.  There are only three routines that one needs
; to write, and specifing the serial port's I/O address, in
; order to make this version work with your own DIY computer.
; Those routines can be found at the end of the source code.
;
; I hope you find good use for this relic. However, I would
; ask that if you do find use for it, please put a reference
; to me in your work. And please, distribute freely.
;*************************************************************

include '../lib/io.asm'


; Modified to work on the TEC-1F using Bit Bang Serial
; TEC 1F Contants
BAUD:	         EQU	 001BH           ; BAUD 4800 delay
KEYBUF:          EQU     00H             ; Keyboard Bit 7 (INPUT)
SCAN:            EQU     01H             ; Segment Scan Bit 6 (OUTPUT)

;TINY BASIC CONSTANTS
;ASCII CONTROL CHARACTERS
SPACE:           EQU     20H             ; Space
TAB:             EQU     09H             ; HORIZONTAL TAB
CTRLC:           EQU     03H             ; Control "C"
CTRLG:           EQU     07H             ; Control "G"
BKSP:            EQU     08H             ; Back space
LF:              EQU     0AH             ; Line feed
VT:              EQU     0BH             ; Vertical tab
CS:              EQU     0CH             ; Clear screen
CR:              EQU     0DH             ; Carriage return
CTRLO:           EQU     0FH             ; Control "O"
CTRLQ:           EQU     11H             ; Control "Q"
CTRLD:           EQU     04H             ; Control "D"
CTRLU:           EQU     15H             ; Control "U"
CTRLZ:           EQU     1AH             ; Control "Z"
ESC:             EQU     1BH             ; Escape
DEL:             EQU     7FH             ; Delete

;LOCAL VARIABLES
OCSW:            EQU     2000H           ;SWITCH FOR OUTPUT
CURRNT:          EQU     OCSW+1          ;POINTS FOR OUTPUT
STKGOS:          EQU     OCSW+3          ;SAVES SP IN 'GOSUB'
VARNXT:          EQU     OCSW+5          ;TEMP STORAGE
STKINP:          EQU     OCSW+7          ;SAVES SP IN 'INPUT'
LOPVAR:          EQU     OCSW+9          ;'FOR' LOOP SAVE AREA
LOPINC:          EQU     OCSW+11         ;INCREMENT
LOPLMT:          EQU     OCSW+13         ;LIMIT
LOPLN:           EQU     OCSW+15         ;LINE NUMBER
LOPPT:           EQU     OCSW+17         ;TEXT POINTER
RANPNT:          EQU     OCSW+19         ;RANDOM NUMBER POINTER
TXTUNF:          EQU     OCSW+21         ;->UNFILLED TEXT AREA
TXTBGN:          EQU     OCSW+23         ;TEXT SAVE AREA BEGINS
TXTEND:          EQU     3E00H           ;TEXT SAVE AREA ENDS

;STACK, VARIABLES AND INPUT BUFFER
VARBGN:          EQU     3E00H           ;VARIABLE LOCATION (SIZE OF 55)
BUFFER:          EQU     3E37H           ;INPUT BUFFER (SIZE OF 80)
BUFEND:          EQU     3E87H           ;BUFFER END
STKLMT:          EQU     3E88H           ;TOP LIMIT FOR STACK
STACK:           EQU     3EFFH           ;STACK

;*************************************************************
; *** ZERO PAGE SUBROUTINES ***
;
; THE Z80 INSTRUCTION SET ALLOWS FOR 8 ROUTINES IN LOW MEMORY
; THAT MAY BE CALLED BY RST 00H, 08H, 10H, 18H, 20H, 28H, 30H,
; AND 38H.  THIS IS A ONE BYTE INSTRUCTION, AND IS FUNCTIONALLY
; SIMILAR TO THE THREE BYTE INSTRUCTION 'CALL XXXX'. TINY BASIC
; WILL USE THE RST INSTRUCTION FOR THE 7 MOST FREQUENTLY USED
; SUBROUTINES. TWO OTHER SUBROUTINES (CRLF & TSTNUM) ARE ALSO
; IN THIS SECTION. THEY CAN BE REACHED WITH 'CALL'.
;*************************************************************

BAS_LOC:           EQU    0x3000          ; LOCATION

DWA     MACRO WHERE
        DB   (WHERE >> 8) + 128
        DB   (WHERE & 0FFH)
        ENDM

        ORG    0x000
        JP     START
        defs   BAS_LOC-3,00
        
        ORG    BAS_LOC
START:
        LD SP,STACK                     ;*** COLD START ***
        JP INIT

RST08:  EX (SP),HL                      ;*** TSTC OR RST 08H ***
        CALL RST28                      ;IGNORE BLANKS AND
        CP (HL)                         ;TEST CHARACTER
        JP TC1                          ;REST OF THIS IS AT TC1

CRLF:
        LD A,CR                         ;*** CRLF ***

RST10:  PUSH AF                         ;*** OUTC OR RST 10H ***
        LD A,(OCSW)                     ;PRINT CHARACTER ONLY
        OR A                            ;IF OCSW SWITCH IS ON
        JP OUTC             ;REST OF THIS AT OUTC

RST18:  CALL EXPR2                      ;*** EXPR OR RST 18H ***
        PUSH HL                         ;EVALUATE AN EXPRESSION
        JP EXPR1                        ;REST OF IT AT EXPR1

RST20:  LD A,H                          ;*** COMP OR RST 20H ***
        CP D                            ;COMPARE HL WITH DE
        RET NZ                          ;RETURN CORRECT C AND
        LD A,L                          ;Z FLAGS
        CP E                            ;BUT OLD A IS LOST
        RET

RST28:  LD A,(DE)                       ;*** IGNBLK/RST 28H ***
        CP SPACE                        ;IGNORE BLANKS
        RET NZ                          ;IN TEXT (WHERE DE->)
        INC DE                          ;AND RETURN THE FIRST
        JR RST28                        ;NON-BLANK CHAR. IN A

RST30:  POP AF                          ;*** FINISH/RST 30H ***
        CALL FIN                        ;CHECK END OF COMMAND
        JP QWHAT                        ;PRINT "WHAT?" IF WRONG

RST38:  CALL RST28                      ;*** TSTV OR RST 38H ***
        SUB 40H                         ;TEST VARIABLES
        RET C                           ;C:NOT A VARIABLE
        JR NZ,TV1                       ;NOT "@" ARRAY
        INC DE                          ;IT IS THE "@" ARRAY
        CALL PARN                       ;@ SHOULD BE FOLLOWED
        ADD HL,HL                       ;BY (EXPR) AS ITS INDEX
        JR C,QHOW                       ;IS INDEX TOO BIG?
        PUSH DE                         ;WILL IT OVERWRITE
        EX DE,HL                        ;TEXT?
        CALL SIZE                       ;FIND SIZE OF FREE
        CALL RST20                      ;AND CHECK THAT
        JP C,ASORRY                     ;IF SO, SAY "SORRY"
        LD HL,VARBGN                    ;IF NOT GET ADDRESS
        CALL SUBDE                      ;OF @(EXPR) AND PUT IT
        POP DE                          ;IN HL
        RET                             ;C FLAG IS CLEARED

TV1:
        CP 1BH                          ;NOT @, IS IT A TO Z?
        CCF                             ;IF NOT RETURN C FLAG
        RET C
        INC DE                          ;IF A THROUGH Z
        LD HL,VARBGN                    ;COMPUTE ADDRESS OF
        RLCA                            ;THAT VARIABLE
        ADD A,L                         ;AND RETURN IT IN HL
        LD L,A                          ;WITH C FLAG CLEARED
;        XOR A
;        ADC A,H
;        LD H,A
        RET

TC1:
        INC HL                          ;COMPARE THE BYTE THAT
        JR Z,TC2                        ;FOLLOWS THE RST INST.
        PUSH BC                         ;WITH THE TEXT (DE->)
        LD C,(HL)                       ;IF NOT =, ADD THE 2ND
        LD B,00H                        ;BYTE THAT FOLLOWS THE
        ADD HL,BC                       ;RST TO THE OLD PC
        POP BC                          ;I.E., DO A RELATIVE
        DEC DE                          ;JUMP IF NOT =

TC2:
        INC DE                          ;IF =, SKIP THOSE BYTES
        INC HL                          ;AND CONTINUE
        EX (SP),HL
        RET

TSTNUM:
        LD HL,0000H                     ;*** TSTNUM ***
        LD B,H                          ;TEST IF THE TEXT IS
        CALL RST28                      ;A NUMBER

TN1:
        CP "0"                          ;IF NOT, RETURN 0 IN
        RET C                           ;B AND HL
        CP ":"                          ;IF NUMBERS, CONVERT
        RET NC                          ;TO BINARY IN HL AND
        LD A,0F0H                       ;SET B TO # OF DIGITS
        AND H                           ;IF H>255, THERE IS NO
        JR NZ,QHOW                      ;ROOM FOR NEXT DIGIT
        INC B                           ;B COUNTS # OF DIGITS
        PUSH BC
        LD B,H                          ;HL=10*HL+(NEW DIGIT)
        LD C,L
        ADD HL,HL                       ;WHERE 10* IS DONE BY
        ADD HL,HL                       ;SHIFT AND ADD
        ADD HL,BC
        ADD HL,HL
        LD A,(DE)                       ;AND (DIGIT) IS FROM
        INC DE                          ;STRIPPING THE ASCII
        AND 0FH                         ;CODE
        ADD A,L
        LD L,A
        LD A,00H                        ;DO THIS TO MAINTAIN CARRY FLAG
        ADC A,H                         ;WHILE SETTING A TO ZERO
        LD H,A
        POP BC
        LD A,(DE)                       ;DO THIS DIGIT AFTER
        JP P,TN1                        ;DIGIT. S SAYS OVERFLOW

QHOW:
        PUSH DE                         ;*** ERROR "HOW?" ***
AHOW:
        LD DE,HOW
        JP ERROR_ROUTINE


HOW:    DB "HOW?",CR
OK:     DB "OK",CR
WHAT:   DB "WHAT?",CR
SORRY:  DB "SORRY",CR

;*************************************************************
;
; *** MAIN ***
;
; THIS IS THE MAIN LOOP THAT COLLECTS THE TINY BASIC PROGRAM
; AND STORES IT IN THE MEMORY.
;
; AT START, IT PRINTS OUT "(CR)OK(CR)", AND INITIALIZES THE
; STACK AND SOME OTHER INTERNAL VARIABLES.  THEN IT PROMPTS
; ">" AND READS A LINE.  IF THE LINE STARTS WITH A NON-ZERO
; NUMBER, THIS NUMBER IS THE LINE NUMBER.  THE LINE NUMBER
; (IN 16 BIT BINARY) AND THE REST OF THE LINE (INCLUDING CR)
; IS STORED IN THE MEMORY.  IF A LINE WITH THE SAME LINE
; NUMBER IS ALREADY THERE, IT IS REPLACED BY THE NEW ONE.  IF
; THE REST OF THE LINE CONSISTS OF A CR ONLY, IT IS NOT STORED
; AND ANY EXISTING LINE WITH THE SAME LINE NUMBER IS DELETED.
;
; AFTER A LINE IS INSERTED, REPLACED, OR DELETED, THE PROGRAM
; LOOPS BACK AND ASKS FOR ANOTHER LINE.  THIS LOOP WILL BE
; TERMINATED WHEN IT READS A LINE WITH ZERO OR NO LINE
; NUMBER; AND CONTROL IS TRANSFERED TO "DIRECT".
;
; TINY BASIC PROGRAM SAVE AREA STARTS AT THE MEMORY LOCATION
; LABELED "TXTBGN" AND ENDS AT "TXTEND".  WE ALWAYS FILL THIS
; AREA STARTING AT "TXTBGN", THE UNFILLED PORTION IS POINTED
; BY THE CONTENT OF A MEMORY LOCATION LABELED "TXTUNF".
;
; THE MEMORY LOCATION "CURRNT" POINTS TO THE LINE NUMBER
; THAT IS CURRENTLY BEING INTERPRETED.  WHILE WE ARE IN
; THIS LOOP OR WHILE WE ARE INTERPRETING A DIRECT COMMAND
; (SEE NEXT SECTION). "CURRNT" SHOULD POINT TO A 0.
;*************************************************************

RSTART:
        LD SP,STACK

ST1:
        LD A,0FFH
        LD (OCSW),A                     ;ENSURE OUTPUT IS ON
        CALL CRLF                       ;AND JUMP TO HERE
        LD DE,OK                        ;DE->STRING
        SUB A                           ;A=0
        CALL PRTSTG                     ;PRINT STRING UNTIL CR
        LD HL,ST2+1                     ;LITERAL 0 STORED AT STG+1
        LD (CURRNT),HL                  ;CURRENT->LINE # = 0

ST2:
        LD HL,0000H
        LD (LOPVAR),HL
        LD (STKGOS),HL

ST3:
        LD A,'>'                        ;PROMPT '>' AND
        CALL GETLN                      ;READ A LINE
        PUSH DE                         ;DE->END OF LINE
        LD DE,BUFFER                    ;DE->BEGINNING OF LINE
        CALL TSTNUM                     ;TEST IF IT IS A NUMBER
        CALL RST28
        LD A,H                          ;HL=VALUE OF THE # OR
        OR L                            ;0 IF NO # WAS FOUND
        POP BC                          ;BC->END OF LINE
        JP Z,DIRECT
        DEC DE                          ;BACKUP DE AND SAVE
        LD A,H                          ;VALUE OF LINE # THERE
        LD (DE),A
        DEC DE
        LD A,L
        LD (DE),A
        PUSH BC                         ;BC,DE->BEGIN, END
        PUSH DE
        LD A,C
        SUB E

        PUSH AF                         ;A=# OF BYTES IN LINE
        CALL FNDLN                      ;FIND THIS LINE IN SAVE
        PUSH DE                         ;AREA, DE->SAVE AREA
        JR NZ,ST4                       ;NZ:NOT FOUND, INSERT
        PUSH DE                         ;Z:FOUND, DELETE IT
        CALL FNDNXT                     ;FIND NEXT LINE
                                        ;DE->NEXT LINE
        POP BC                          ;BC->LINE TO BE DELETED
        LD HL,(TXTUNF)                  ;HL->UNFILLED SAVE AREA
        CALL MVUP                       ;MOVE UP TO DELETE
        LD H,B                          ;TXTUNF->UNFILLED ARA
        LD L,C
        LD (TXTUNF),HL                  ;UPDATE

ST4:
        POP BC                          ;GET READY TO INSERT
        LD HL,(TXTUNF)                  ;BUT FIRST CHECK IF
        POP AF                          ;THE LENGTH OF NEW LINE
        PUSH HL                         ;IS 3 (LINE # AND CR)
        CP CTRLC                        ;THEN DO NOT INSERT
        JR Z,RSTART                     ;MUST CLEAR THE STACK
        ADD A,L                         ;COMPUTE NEW TXTUNF
        LD L,A
        LD A,00H
        ADC A,H
        LD H,A                          ;HL->NEW UNFILLED AREA
        LD DE,TXTEND                    ;CHECK TO SEE IF THERE
        CALL RST20                         ;IS ENOUGH SPACE
        JP NC,QSORRY                    ;SORRY, NO ROOM FOR IT
        LD (TXTUNF),HL                  ;OK, UPDATE TXTUNF
        POP DE                          ;DE->OLD UNFILLED AREA
        CALL MVDOWN
        POP DE                          ;DE->BEGIN, HL->END
        POP HL
        CALL MVUP                       ;MOVE NEW LINE TO SAVE
        JR ST3                          ;AREA

;*************************************************************
;
; WHAT FOLLOWS IS THE CODE TO EXECUTE DIRECT AND STATEMENT
; COMMANDS.  CONTROL IS TRANSFERED TO THESE POINTS VIA THE
; COMMAND TABLE LOOKUP CODE OF 'DIRECT' AND 'EXEC' IN LAST
; SECTION.  AFTER THE COMMAND IS EXECUTED, CONTROL IS
; TRANSFERED TO OTHERS SECTIONS AS FOLLOWS:
;
; FOR 'LIST', 'NEW', AND 'STOP': GO BACK TO 'RSTART'
; FOR 'RUN': GO EXECUTE THE FIRST STORED LINE IF ANY, ELSE
; GO BACK TO 'RSTART'.
; FOR 'GOTO' AND 'GOSUB': GO EXECUTE THE TARGET LINE.
; FOR 'RETURN' AND 'NEXT': GO BACK TO SAVED RETURN LINE.
; FOR ALL OTHERS: IF 'CURRENT' -> 0, GO TO 'RSTART', ELSE
; GO EXECUTE NEXT COMMAND.  (THIS IS DONE IN 'FINISH'.)
;*************************************************************
;
; *** NEW *** STOP *** RUN (& FRIENDS) *** & GOTO ***
;
; 'NEW(CR)' SETS 'TXTUNF' TO POINT TO 'TXTBGN'
;
; 'STOP(CR)' GOES BACK TO 'RSTART'
;
; 'RUN(CR)' FINDS THE FIRST STORED LINE, STORE ITS ADDRESS (IN
; 'CURRENT'), AND START EXECUTE IT.  NOTE THAT ONLY THOSE
; COMMANDS IN TAB2 ARE LEGAL FOR STORED PROGRAM.
;
; THERE ARE 3 MORE ENTRIES IN 'RUN':
; 'RUNNXL' FINDS NEXT LINE, STORES ITS ADDR. AND EXECUTES IT.
; 'RUNTSL' STORES THE ADDRESS OF THIS LINE AND EXECUTES IT.
; 'RUNSML' CONTINUES THE EXECUTION ON SAME LINE.
;
; 'GOTO EXPR(CR)' EVALUATES THE EXPRESSION, FIND THE TARGET
; LINE, AND JUMP TO 'RUNTSL' TO DO IT.
;*************************************************************

NEW:
        CALL ENDCHK                     ;*** NEW(CR) ***
        LD HL,TXTBGN
        LD (TXTUNF),HL
STOP:
        CALL ENDCHK                     ;*** STOP(CR) ***
        JP RSTART
RUN:
        CALL ENDCHK                     ;*** RUN(CR) ***
        LD DE,TXTBGN                    ;FIRST SAVED LINE
RUNNXL:
        LD HL,00H                       ;*** RUNNXL ***
        CALL FNDLP                      ;FIND WHATEVER LINE #
        JP C,RSTART                     ;C:PASSED TXTUNF, QUIT
RUNTSL:
        EX DE,HL                        ;*** RUNTSL ***
        LD (CURRNT),HL                  ;SET 'CURRENT'->LINE #
        EX DE,HL
        INC DE                          ;BUMP PASS LINE #
        INC DE
RUNSML:
        CALL CHKIO                      ;*** RUNSML ***
        LD HL,TAB2-1                    ;FIND COMMAND IN TAB2
        JP EXEC                         ;AND EXECUTE IT
GOTO:
        CALL RST18                         ;*** GOTO EXPR ***
        PUSH DE                         ;SAVE FOR ERROR ROUTINE
        CALL ENDCHK                     ;MUST FIND A CR
        CALL FNDLN                      ;FIND THE TARGET LINE
        JP NZ,AHOW                      ;NO SUCH LINE #
        POP AF                          ;CLEAR THE PUSH DE
        JR RUNTSL                       ;GO DO IT

;*************************************************************
;
; *** LIST *** & PRINT ***
;
; LIST HAS TWO FORMS:
; 'LIST(CR)' LISTS ALL SAVED LINES
; 'LIST #(CR)' START LIST AT THIS LINE #
; YOU CAN STOP THE LISTING BY CONTROL C KEY
;
; PRINT COMMAND IS 'PRINT ....;' OR 'PRINT ....(CR)'
; WHERE '....' IS A LIST OF EXPRESIONS, FORMATS, BACK-
; ARROWS, AND STRINGS.  THESE ITEMS ARE SEPERATED BY COMMAS.
;
; A FORMAT IS A POUND SIGN FOLLOWED BY A NUMBER.  IT CONTROLS
; THE NUMBER OF SPACES THE VALUE OF A EXPRESION IS GOING TO
; BE PRINTED.  IT STAYS EFFECTIVE FOR THE REST OF THE PRINT
; COMMAND UNLESS CHANGED BY ANOTHER FORMAT.  IF NO FORMAT IS
; SPECIFIED, 6 POSITIONS WILL BE USED.
;
; IF A & IS THE FIRST CHARACTER, THE NUMBER FOLLOWING IS 
; PRINTED AS HEXADECIMAL.  IF NUMBER IS GREATER THAN 255
; THEN H IS PRINTED FIRST THEN L WITH NO SPACE BETWEEN
;
; IF A % IS THE FIRST CHARACTER, THE NUMBER IS PRINTED
; AS AN ASCII CHARCTER.  IF THE CARACTER IS NOT PRINTABLE
; A '.' WILL BE PRINTED.
;
; IF A $ IS THE FIRST CHARACTER, THE NUMBER IS PRINTED
; AS AN ASCII CHARCTER BUT ALL ASCII CHARACTERS ARE PRINTED
; BETWEEN 0 AND 128
;
; A STRING IS QUOTED IN A PAIR OF SINGLE QUOTES OR A PAIR OF
; DOUBLE QUOTES.
;
; A UNDERSCORE MEANS GENERATE A (CR) WITHOUT (LF)
;
; A (CRLF) IS GENERATED AFTER THE ENTIRE LIST HAS BEEN
; PRINTED OR IF THE LIST IS A NULL LIST.  HOWEVER IF THE LIST
; ENDED WITH A COMMA, NO (CRLF) IS GENERATED.
;*************************************************************

LIST:
        CALL TSTNUM                     ;TEST IF THERE IS A #
        CALL ENDCHK                     ;IF NO # WE GET A 0
        CALL FNDLN                      ;FIND THIS OR NEXT LINE
LS1:
        JP C,RSTART                     ;C:PASSED TXTUNF
        CALL PRTLN                      ;PRINT THE LINE
        CALL CHKIO                      ;STOP IF HIT CONTROL-C
        CALL FNDLP                      ;FIND NEXT LINE
        JR LS1                          ;AND LOOP BACK
PRINT:
        LD C,06H                        ;C = # OF SPACES
        CALL RST08                         ;F NULL LIST & ";"
        DB ";"
        DB PR2-$-1
        CALL CRLF                       ;GIVE CR-LF AND
        JR RUNSML                       ;CONTINUE SAME LINE
PR2:
        CALL RST08                         ;IF NULL LIST (CR)
        DB CR
        DB PR0-$-1
        CALL CRLF                       ;ALSO GIVE CR-LF AND
        JR RUNNXL                       ;GO TO NEXT LINE
PR0:
        CALL RST08                         ;ELSE IS IT FORMAT?
        DB '#'
        DB PR9-$-1
        CALL RST18                         ;YES, EVALUATE EXPR.
        LD C,L                             ;AND SAVE IT IN C
        JR PR3                             ;LOOK FOR MORE TO PRINT
PR9:
        CALL RST08                         ;ELSE IS IT PRINT AS HEXIDECIMAL?
        DB '&'
        DB PR10-$-1
        CALL RST18                         ;YES, EVALUATE EXPR.
        CALL PRTHEX
        JR PR3
PR10:
        CALL RST08                         ;ELSE IS IT PRINT AS ASCII?
        DB '%'                             ;FOR ASCII PRINTING?
        DB PR11-$-1
        CALL RST18                         ;YES, EVALUATE EXPR.
        CALL PRTASC                        ;PRINT ONLY PRINTABLE ASCII        
        JR PR3
PR11:
        CALL RST08                         ;ELSE IS IT CHR$?
        DB '$'
        DB PR1-$-1
        CALL RST18                         ;YES, EVALUATE EXPR.
        CALL PRTCHR                        ;PRINT THE ASCII VALUE
        JR PR3
PR1:
        CALL QTSTG                      ;OR IS IT A STRING?
        JR PR8                          ;IF NOT, MUST BE EXPR.
PR3:
        CALL RST08                         ;IF ",", GO FIND NEXT
        DB ','
        DB PR6-$-1
        CALL FIN                        ;IN THE LIST.
        JR PR0                          ;LIST CONTINUES
PR6:
        CALL CRLF                       ;LIST ENDS
        CALL RST30
PR8:
        CALL RST18                         ;EVALUATE THE EXPR
        PUSH BC
        CALL PRTNUM                     ;PRINT THE VALUE
        POP BC
        JR PR3                          ;MORE TO PRINT?

;*************************************************************
;
; *** GOSUB *** & RETURN ***
;
; 'GOSUB EXPR;' OR 'GOSUB EXPR (CR)' IS LIKE THE 'GOTO'
; COMMAND, EXCEPT THAT THE CURRENT TEXT POINTER, STACK POINTER
; ETC. ARE SAVE SO THAT EXECUTION CAN BE CONTINUED AFTER THE
; SUBROUTINE 'RETURN'.  IN ORDER THAT 'GOSUB' CAN BE NESTED
; (AND EVEN RECURSIVE), THE SAVE AREA MUST BE STACKED.
; THE STACK POINTER IS SAVED IN 'STKGOS', THE OLD 'STKGOS' IS
; SAVED IN THE STACK.  IF WE ARE IN THE MAIN ROUTINE, 'STKGOS'
; IS ZERO (THIS WAS DONE BY THE "MAIN" SECTION OF THE CODE),
; BUT WE STILL SAVE IT AS A FLAG FOR NO FURTHER 'RETURN'S.
;
; 'RETURN(CR)' UNDOS EVERYTHING THAT 'GOSUB' DID, AND THUS
; RETURN THE EXECUTION TO THE COMMAND AFTER THE MOST RECENT
; 'GOSUB'.  IF 'STKGOS' IS ZERO, IT INDICATES THAT WE
; NEVER HAD A 'GOSUB' AND IS THUS AN ERROR.
;*************************************************************

GOSUB:
        CALL PUSHA                      ;SAVE THE CURRENT "FOR"
        CALL RST18                         ;PARAMETERS
        PUSH DE                         ;AND TEXT POINTER
        CALL FNDLN                      ;FIND THE TARGET LINE
        JP NZ,AHOW                      ;NOT THERE. SAY "HOW?"
        LD HL,(CURRNT)                  ;FOUND IT, SAVE OLD.
        PUSH HL                         ;'CURRNT' OLD 'STKGOS'
        LD HL,(STKGOS)
        PUSH HL
        LD HL,0000H                     ;AND LOAD NEW ONES
        LD (LOPVAR),HL
        ADD HL,SP
        LD (STKGOS),HL
        JP RUNTSL                       ;THEN RUN THAT LINE
RETURN:
        CALL ENDCHK                     ;THERE MUST BE A CR
        LD HL,(STKGOS)                  ;OLD STACK POINTER
        LD A,H                          ;0 MEANS NOT EXIST
        OR L
        JP Z,QWHAT                      ;SO, WE SAY: "WHAT?"
        LD SP,HL                        ;ELSE, RESTORE IT
        POP HL
        LD (STKGOS),HL                  ;AND THE OLD "STKGOS"
        POP HL
        LD (CURRNT),HL                  ;AND THE OLD 'CURRNT'
        POP DE                          ;OLD TEXT POINTER
        CALL POPA                       ;OLD "FOR" PARAMETERS
        CALL RST30                         ;AND WE ARE BACK HOME

;*************************************************************
;
; *** FOR *** & NEXT ***
;
; 'FOR' HAS TWO FORMS:
; 'FOR VAR=EXP1 TO EXP2 STEP EXP3' AND 'FOR VAR=EXP1 TO EXP2'
; THE SECOND FORM MEANS THE SAME THING AS THE FIRST FORM WITH
; EXP3=1.  (I.E., WITH A STEP OF +1.)
; TBI WILL FIND THE VARIABLE VAR, AND SET ITS VALUE TO THE
; CURRENT VALUE OF EXP1.  IT ALSO EVALUATES EXP2 AND EXP3
; AND SAVE ALL THESE TOGETHER WITH THE TEXT POINTER ETC. IN
; THE 'FOR' SAVE AREA, WHICH CONSISTS OF 'LOPVAR', 'LOPINC',
; 'LOPLMT', 'LOPLN', AND 'LOPPT'.  IF THERE IS ALREADY SOME-
; THING IN THE SAVE AREA (THIS IS INDICATED BY A NON-ZERO
; 'LOPVAR'), THEN THE OLD SAVE AREA IS SAVED IN THE STACK
; BEFORE THE NEW ONE OVERWRITES IT.
; TBI WILL THEN DIG IN THE STACK AND FIND OUT IF THIS SAME
; VARIABLE WAS USED IN ANOTHER CURRENTLY ACTIVE 'FOR' LOOP.
; IF THAT IS THE CASE, THEN THE OLD 'FOR' LOOP IS DEACTIVATED.
; (PURGED FROM THE STACK..)
;
; 'NEXT VAR' SERVES AS THE LOGICAL (NOT NECESSARILLY PHYSICAL)
; END OF THE 'FOR' LOOP.  THE CONTROL VARIABLE VAR. IS CHECKED
; WITH THE 'LOPVAR'.  IF THEY ARE NOT THE SAME, TBI DIGS IN
; THE STACK TO FIND THE RIGHT ONE AND PURGES ALL THOSE THAT
; DID NOT MATCH.  EITHER WAY, TBI THEN ADDS THE 'STEP' TO
; THAT VARIABLE AND CHECK THE RESULT WITH THE LIMIT.  IF IT
; IS WITHIN THE LIMIT, CONTROL LOOPS BACK TO THE COMMAND
; FOLLOWING THE 'FOR'.  IF OUTSIDE THE LIMIT, THE SAVE AREA
; IS PURGED AND EXECUTION CONTINUES.
;*************************************************************

FOR:
        CALL PUSHA                      ;SAVE THE OLD SAVE AREA
        CALL SETVAL                     ;SET THE CONTROL VAR.
        DEC HL                          ;HL IS ITS ADDRESS
        LD (LOPVAR),HL                  ;SAVE THAT
        LD HL,TAB5-1                    ;USE 'EXEC' TO LOOK
        JP EXEC                         ;FOR THE WORK 'TO'
FR1:
        CALL RST18                         ;EVALUATE THE LIMITE
        LD (LOPLMT),HL                  ;SAVE THAT
        LD HL,TAB6-1                    ;USE 'EXEC' TO LOOK
        JP EXEC                         ;FOR THE WORD 'STEP'
FR2:
        CALL RST18                         ;FOUND IT, GET STEP
        JR FR4
FR3:
        LD HL,0001H                     ;NOT FOUND, SET TO 1
FR4:
        LD (LOPINC),HL                  ;SAVE THAT TOO
FR5:
        LD HL,(CURRNT)                  ;SAVE CURRENT LINE #
        LD (LOPLN),HL
        EX DE,HL                        ;AND TEXT POINTER
        LD (LOPPT),HL
        LD BC,0AH                       ;DIG INTO STACK TO
        LD HL,(LOPVAR)                  ;FIND 'LOPVAR'
        EX DE,HL
        LD H,B
        LD L,B                          ;HL=0 NOW
        ADD HL,SP                       ;HERE IS THE STACK
FR7:
        ADD HL,BC                       ;EACH LEVEL IS 10 DEEP - DIS = 09
        LD A,L
        ADD A,C
        JR C,FR8
        LD A,(HL)
        DEC HL
        CP D                            ;SAME AS THIS ONE?
        JR NZ,FR7
        LD A,(HL)                       ;THE OTHER HALF?
        CP E
        JR NZ,FR7
        EX DE,HL                        ;YES, FOUND ONE
        LD HL,0000H
        ADD HL,SP                       ;TRY TO MOVE SP
        LD B,H
        LD C,L
        LD HL,000AH
        ADD HL,DE
        CALL MVDOWN                     ;AND PURGE 10 WORDS
        LD SP,HL                        ;IN THE STACK
FR8:
        LD HL,(LOPPT)                   ;JOB DONE, RESTORE DE
        EX DE,HL
        CALL RST30                         ;AND CONTINUE
;
NEXT:
        CALL RST38                         ;GET ADDRESS OF VAR.
        JP C,QWHAT                      ;NO VARIABLE, "WHAT?"
        LD (VARNXT),HL                  ;YES, SAVE IT
NX0:
        PUSH DE                         ;SAVE TEXT POINTER
        EX DE,HL
        LD HL,(LOPVAR)                  ;GET VAR. IN 'FOR'
        LD A,H
        OR L                            ;0 SAYS NEVER HAD ONE
        JP Z,AWHAT                      ;SO WE ASK: "WHAT?"
        CALL RST20                         ;ELSE WE CHECK THEM
        JR Z,NX3                        ;OK, THEY AGREE
        POP DE                          ;NO, LET'S SEE
        CALL POPA                       ;PURGE CURRENT LOOP
        LD HL,(VARNXT)                  ;AND POP ONE LEVEL
        JR NX0                          ;GO CHECK AGAIN
NX3:
        LD E,(HL)                       ;COME HERE WHEN AGREED
        INC HL
        LD D,(HL)                       ;DE=VALUE OF VAR.
        LD HL,(LOPINC)
        PUSH HL
        LD A,H
        XOR D
        LD A,D
        ADD HL,DE                       ;ADD ONE STEP
        JP M,NX4
        XOR H
        JP M,NX5
NX4:
        EX DE,HL
        LD HL,(LOPVAR)                  ;PUT IT BACK
        LD (HL),E
        INC HL
        LD (HL),D
        LD HL,(LOPLMT)                  ;HL->LIMIT
        POP AF                          ;OLD HL
        OR A
        JP P,NX1                        ;STEP > 0
        EX DE,HL                        ;STEP < 0
NX1:
        CALL CKHLDE                     ;COMPARE WITH LIMIT
        POP DE                          ;RESTORE TEXT POINTER
        JR C,NX2                        ;OUTSIDE LIMIT
        LD HL,(LOPLN)                   ;WITHIN LIMIT, GO
        LD (CURRNT),HL                  ;BACK TO THE SAVED
        LD HL,(LOPPT)                   ;'CURRNT' AND TEXT
        EX DE,HL                        ;POINTER
        CALL RST30
NX5:
        POP HL
        POP DE
NX2:
        CALL POPA                       ;PURGE THIS LOOP
        CALL RST30

;*************************************************************
;
; *** REM *** IF *** INPUT *** & LET (& DEFLT) ***
;
; 'REM' CAN BE FOLLOWED BY ANYTHING AND IS IGNORED BY TBI.
; TBI TREATS IT LIKE AN 'IF' WITH A FALSE CONDITION.
;
; 'IF' IS FOLLOWED BY AN EXPR. AS A CONDITION AND ONE OR MORE
; COMMANDS (INCLUDING OTHER 'IF'S) SEPERATED BY SEMI-COLONS.
; NOTE THAT THE WORD 'THEN' IS NOT USED.  TBI EVALUATES THE
; EXPR. IF IT IS NON-ZERO, EXECUTION CONTINUES.  IF THE
; EXPR. IS ZERO, THE COMMANDS THAT FOLLOWS ARE IGNORED AND
; EXECUTION CONTINUES AT THE NEXT LINE.
;
; 'INPUT' COMMAND IS LIKE THE 'PRINT' COMMAND, AND IS FOLLOWED
; BY A LIST OF ITEMS.  IF THE ITEM IS A STRING IN SINGLE OR
; DOUBLE QUOTES, OR IS A UNDERSCORE, IT HAS THE SAME EFFECT AS
; IN 'PRINT'.  IF AN ITEM IS A VARIABLE, THIS VARIABLE NAME IS
; PRINTED OUT FOLLOWED BY A COLON.  THEN TBI WAITS FOR AN
; EXPR. TO BE TYPED IN.  THE VARIABLE IS THEN SET TO THE
; VALUE OF THIS EXPR.  IF THE VARIABLE IS PROCEDED BY A STRING
; (AGAIN IN SINGLE OR DOUBLE QUOTES), THE STRING WILL BE
; PRINTED FOLLOWED BY A COLON.  TBI THEN WAITS FOR INPUT EXPR.
; AND SET THE VARIABLE TO THE VALUE OF THE EXPR.
;
; IF THE INPUT EXPR. IS INVALID, TBI WILL PRINT "WHAT?",
; "HOW?" OR "SORRY" AND REPRINT THE PROMPT AND REDO THE INPUT.
; THE EXECUTION WILL NOT TERMINATE UNLESS YOU TYPE CONTROL-C.
; THIS IS HANDLED IN 'INPERR'.
;
; 'LET' IS FOLLOWED BY A LIST OF ITEMS SEPERATED BY COMMAS.
; EACH ITEM CONSISTS OF A VARIABLE, AN EQUAL SIGN, AND AN EXPR.
; TBI EVALUATES THE EXPR. AND SET THE VARIABLE TO THAT VALUE.
; TBI WILL ALSO HANDLE 'LET' COMMAND WITHOUT THE WORD 'LET'.
; THIS IS DONE BY 'DEFLT'.
;*************************************************************

REM:
        LD HL,0000H                     ;*** REM ***
        JR IF1                       
IFF:
        CALL RST18                         ;*** IF ***
IF1:
        LD A,H                          ;IS THE EXPR.=0?
        OR L
        JP NZ,RUNSML                    ;NO, CONTINUE
        CALL FNDSKP                     ;YES, SKIP REST OF LINE
        JP NC,RUNTSL                    ;AND RUN THE NEXT LINE
        JP RSTART                       ;IF NO NEXT, RE-START
INPERR:
        LD HL,(STKINP)                  ;*** INPERR ***
        LD SP,HL                        ;RESTORE OLD SP
        POP HL                          ;AND OLD 'CURRNT'
        LD (CURRNT),HL
        POP DE                          ;AND OLD TEXT POINTER
        POP DE                          ;REDO INPUT
INPUT:                                  ;*** INPUT ***
IP1:
        PUSH DE                         ;SAVE IN CASE OF ERROR
        CALL QTSTG                      ;IS NEXT ITEM A STRING?
        JR IP2                          ;NO
        CALL RST38                         ;YES, BUT FOLLOWED BY A
        JR C,IP4                        ;VARIABLE? NO.
        JR IP3                          ;YES. INPUT VARIABLE
IP2:
        PUSH DE                         ;SAVE FOR 'PRTSTG'
        CALL RST38                         ;MUST BE VARIABLE NOW
        JP C,QWHAT                      ;"WHAT?" IT IS NOT?
        LD A,(DE)                       ;GET READY FOR 'PRTSTR'
        LD C,A
        SUB A
        LD (DE),A
        POP DE
        CALL PRTSTG                     ;PRINT STRING AS PROMPT
        LD A,C                          ;RESTORE TEXT
        DEC DE
        LD (DE),A
IP3:
        PUSH DE                         ;SAVE TEXT POINTER
        EX DE,HL
        LD HL,(CURRNT)                  ;ALSO SAVE 'CURRNT'
        PUSH HL
        LD HL,IP1                       ;A NEGATIVE NUMBER
        LD (CURRNT),HL                  ;AS A FLAG
        LD HL,0000H                     ;SAVE SP TOO
        ADD HL,SP
        LD (STKINP),HL
        PUSH DE                         ;OLD HL
        LD A,3AH                        ;PRINT THIS TOO
        CALL GETLN                      ;AND GET A LINE
        LD DE,BUFFER                    ;POINTS TO BUFFER
        CALL RST18                         ;EVALUATE INPUT
        NOP                             ;CAN BE 'CALL ENDCHK'
        NOP
        NOP
        POP DE                          ;OK,GET OLD HL
        EX DE,HL
        LD (HL),E                       ;SAVE VALUE IN VAR.
        INC HL
        LD (HL),D
        POP HL                          ;GET OLD 'CURRNT'
        LD (CURRNT),HL
        POP DE                          ;AND OLD TEXT POINTER
IP4:
        POP AF                          ;PURGE JUNK IN STACK
        CALL RST08                         ;IS NEXT CH. ','?
        DB ','
        DB IP5-$-1
        JR IP1                          ;YES, MORE ITEMS.
IP5:
        CALL RST30
DEFLT:
        LD A,(DE)                       ;***  DEFLT ***
        CP CR                           ;EMPTY LINE IS OK
        JR Z,LT1                        ;ELSE IT IS 'LET'
LET:
        CALL SETVAL                     ;*** LET ***
        CALL RST08                         ;SET VALUE TO VAR
        DB ','                          ;IS NEXT CH. ','?
        DB LT1-$-1                      
        JR LET                          ;ITEM BY ITEM
LT1:
        CALL RST30                         ;UNTIL FINISH
;*************************************************************
;
; *** EXPR ***
;
; 'EXPR' EVALUATES ARITHMETICAL OR LOGICAL EXPRESSIONS.
; <EXPR>::<EXPR2>
;         <EXPR2><REL.OP.><EXPR2>
; WHERE <REL.OP.> IS ONE OF THE OPERATORS IN TAB8 AND THE
; RESULT OF THESE OPERATIONS IS 1 IF TRUE AND 0 IF FALSE.
; <EXPR2>::=(+ OR -)<EXPR3>(+ OR -<EXPR3>)(....)
; WHERE () ARE OPTIONAL AND (....) ARE OPTIONAL REPEATS.
; <EXPR3>::=<EXPR4>(* OR /)<EXPR4>(....)
; <EXPR4>::=<VARIABLE>
;           <FUNCTION>
;           (<EXPR>)
; <EXPR> IS RECURSIVE SO THAT VARIABLE '@' CAN HAVE AN <EXPR>
; AS INDEX, FUNCTIONS CAN HAVE AN <EXPR> AS ARGUMENTS, AND
; <EXPR4> CAN BE AN <EXPR> IN PARANTHESE.
;*************************************************************

EXPR1:
        LD HL,TAB8-1                    ;LOOKUP REL.OP.
        JP EXEC                         ;GO DO IT
XP11:
        CALL XP18                       ;REL.OP.">="
        RET C                           ;NO, RETURN HL=0
        LD L,A                          ;YES, RETURN HL=1
        RET
XP12:
        CALL XP18                       ;REL.OP."#"
        RET Z                           ;FALSE, RETURN HL=0
        LD L,A                          ;TRUE, RETURN HL=1
        RET
XP13:
        CALL XP18                       ;REL.OP.">"
        RET Z                           ;FALSE
        RET C                           ;ALSO FALSE, HL=0
        LD L,A                          ;TRUE, HL=1
        RET
XP14:
        CALL XP18                       ;REL.OP."<="
        LD L,A                          ;SET HL=1
        RET Z                           ;REL. TRUE, RETURN
        RET C
        LD L,H                          ;ELSE SET HL=0
        RET
XP15:
        CALL XP18                       ;REL.OP."="
        RET NZ                          ;FALSE, RETURN HL=0
        LD L,A                          ;ELSE SET HL=1
        RET
XP16:
        CALL XP18                       ;REL.OP."<"
        RET NC                          ;FALSE, RETURN HL=0
        LD L,A                          ;ELSE SET HL=1
        RET
XP17:
        POP HL                          ;NOT .REL.OP
        RET                             ;RETURN HL=<EXPR2>
XP18:
        LD A,C                          ;SUBROUTINE FOR ALL
        POP HL                          ;REL.OP.'S
        POP BC
        PUSH HL                         ;REVERSE TOP OF STACK
        PUSH BC
        LD C,A
        CALL EXPR2                      ;GET 2ND <EXPR2>
        EX DE,HL                        ;VALUE IN DE NOW
        EX (SP),HL                      ;1ST <EXPR2> IN HL
        CALL CKHLDE                     ;COMPARE 1ST WITH 2ND
        POP DE                          ;RESTORE TEXT POINTER
        LD HL,0000H                     ;SET HL=0, A=1
        LD A,01H
        RET
EXPR2:
        CALL RST08                         ;NEGATIVE SIGN?
        DB '-'
        DB XP21-$-1
        LD HL,0000H                     ;YES, FAKE '0-'
        JR XP26                         ;TREAT LIKE SUBTRACT
XP21:
        CALL RST08                         ;POSITIVE SIGN? IGNORE
        DB '+'
        DB XP22-$-1
XP22:
        CALL EXPR3                      ;1ST <EXPR3>
XP23:
        CALL RST08                         ;ADD?
        DB  '+'
        DB XP25-$-1
        PUSH HL                         ;YES, SAVE VALUE
        CALL EXPR3                      ;GET 2ND <EXPR3>
XP24:
        EX DE,HL                        ;2ND IN DE
        EX (SP),HL                      ;1ST IN HL
        LD A,H                          ;COMPARE SIGN
        XOR D
        LD A,D
        ADD HL,DE
        POP DE                          ;RESTORE TEXT POINTER
        JP M,XP23                       ;1ST AND 2ND SIGN DIFFER
        XOR H                           ;1ST AND 2ND SIGN EQUAL
        JP P,XP23                       ;SO IS RESULT
        JP QHOW                         ;ELSE WE HAVE OVERFLOW
XP25:
        CALL RST08                         ;SUBTRACT?
        DB '-'
        DB XP42-$-1
XP26:
        PUSH HL                         ;YES, SAVE 1ST <EXPR3>
        CALL EXPR3                      ;GET 2ND <EXPR3>
        CALL CHGSGN                     ;NEGATE
        JR XP24                         ;AND ADD THEM
;
EXPR3:
        CALL EXPR4                      ;GET 1ST <EXPR4>
XP31:
        CALL RST08                         ;MULTIPLY?
        DB '*'
        DB XP34-$-1
        PUSH HL                         ;YES, SAVE 1ST
        CALL EXPR4                      ;AND GET 2ND <EXPR4>
        LD B,00H                        ;CLEAR B FOR SIGN
        CALL CHKSGN                     ;CHECK SIGN
        EX (SP),HL                      ;1ST IN HL
        CALL CHKSGN                     ;CHECK SIGN OF 1ST
        EX DE,HL
        EX (SP),HL
        LD A,H                          ;IS HL > 255 ?
        OR A
        JR Z,XP32                       ;NO
        LD A,D                          ;YES, HOW ABOUT DE
        OR D
        EX DE,HL                        ;PUT SMALLER IN HL
        JP NZ,AHOW                      ;ALSO >, WILL OVERFLOW
XP32:
        LD A,L                          ;THIS IS DUMB
        LD HL,0000H                     ;CLEAR RESULT
        OR A                            ;ADD AND COUNT
        JR Z,XP35
XP33:
        ADD HL,DE
        JP C,AHOW                       ;OVERFLOW
        DEC A
        JR NZ,XP33
        JR XP35                         ;FINISHED
XP34:
        CALL RST08                         ;DIVIDE?
        DB '/'
        DB XP42-$-1
        PUSH HL                         ;YES, SAVE 1ST <EXPR4>
        CALL EXPR4                      ;AND GET THE SECOND ONE
        LD B,00H                        ;CLEAR B FOR SIGN
        CALL CHKSGN                     ;CHECK SIGN OF 2ND
        EX (SP),HL                      ;GET 1ST IN HL
        CALL CHKSGN                     ;CHECK SIGN OF 1ST
        EX DE,HL
        EX (SP),HL
        EX DE,HL
        LD A,D                          ;DIVIDE BY 0?
        OR E
        JP Z,AHOW                       ;SAY "HOW?"
        PUSH BC                         ;ELSE SAVE SIGN
        CALL DIVIDE                     ;USE SUBROUTINE
        LD H,B                          ;RESULT IN HL NOW
        LD L,C
        POP BC                          ;GET SIGN BACK
XP35:
        POP DE                          ;AND TEXT POINTER
        LD A,H                          ;HL MUST BE +
        OR A
        JP M,QHOW                       ;ELSE IT IS OVERFLOW
        LD A,B
        OR A
        CALL M,CHGSGN                   ;CHANGE SIGN IF NEEDED
        JR XP31                         ;LOOK FOR MORE TERMS
EXPR4:
        LD HL,TAB4-1                    ;FIND FUNCTION IN TAB4
        JP EXEC                         ;AND GO DO IT
XP40:
        CALL RST38                         ;NO, NOT A FUNCTION
        JR C,XP41                       ;NOR A VARIABLE
        LD A,(HL)                       ;VARIABLE
        INC HL
        LD H,(HL)                       ;VALUE IN HL
        LD L,A
        RET
XP41:
        CALL TSTNUM                     ;OR IS IT A NUMBER
        LD A,B                          ;# OF DIGIT
        OR A
        RET NZ                          ;OK
PARN:
        CALL RST08
        DB '('
        DB XP43-$-1
        CALL RST18                         ;"(EXPR)"
        CALL RST08
        DB ')'
        DB XP43-$-1
XP42:
        RET
XP43:
        JP QWHAT                        ;ELSE SAY: "WHAT?"
RND:
        CALL PARN                       ;*** RND(EXPR) ***
        LD A,H                          ;EXPR MUST BE +
        OR A
        JP M,QHOW
        OR L                            ;AND NON-ZERO
        JP Z,QHOW
        PUSH DE                         ;SAVE BOTH
        PUSH HL
        LD HL,(RANPNT)                  ;GET MEMORY AS RANDOM
        LD DE,LSTROM                    ;NUMBER
        CALL RST20
        JR C,RA1                        ;WRAP AROUND IF LAST
        LD HL,START
RA1:
        LD E,(HL)
        INC HL
        LD D,(HL)
        LD (RANPNT),HL
        POP HL
        EX DE,HL
        PUSH BC
        CALL DIVIDE                     ;RND (N)=MOD(M,N)+1
        POP BC
        POP DE
        INC HL
        RET
ABS:
        CALL PARN                       ;*** ABS (EXPR) ***
        DEC DE
        CALL CHKSGN                     ;CHECK SIGN
        INC DE
        RET
PEEK:
        CALL PARN                       ;*** PEEK (EXPR) ***
        LD A,(HL)                       ;GET VALUE STORED AT EXPRESSION
        LD L,A                          ;STORE VALUE BACK INTO HL
        LD H,00H
        RET
SIZE:
        LD HL,(TXTUNF)                  ;*** SIZE ***
        PUSH DE                         ;GET THE NUMBER OF FREE
        EX DE,HL                        ;BYTES BETWEEN 'TXTUNF'
        LD HL,VARBGN                    ;AND 'VARBGN'
        CALL SUBDE
        POP DE
        RET
;*************************************************************
;
; *** DIVIDE *** SUBDE *** CHKSGN *** CHGSGN *** & CKHLDE ***
;
; 'DIVIDE' DIVIDES HL BY DE, RESULT IN BC, REMAINDER IN HL
;
; 'SUBDE' SUBSTRACTS DE FROM HL
;
; 'CHKSGN' CHECKS SIGN OF HL.  IF +, NO CHANGE.  IF -, CHANGE
; SIGN AND FLIP SIGN OF B.
;
; 'CHGSGN' CHECKS SIGN N OF HL AND B UNCONDITIONALLY.
;
; 'CKHLDE' CHECKS SIGN OF HL AND DE.  IF DIFFERENT, HL AND DE
; ARE INTERCHANGED.  IF SAME SIGN, NOT INTERCHANGED.  EITHER
; CASE, HL DE ARE THEN COMPARED TO SET THE FLAGS.
;*************************************************************

DIVIDE:
        PUSH HL                         ;*** DIVIDE ***
        LD L,H                          ;DIVIDE H BY DE
        LD H,00H
        CALL DV1
        LD B,C                          ;SAVE RESULT IN B
        LD A,L                          ;(REMAINDER+L)/DE
        POP HL
        LD H,A
DV1:
        LD C,0FFH                       ;RESULT IN C
DV2:
        INC C                           ;DUMB ROUTINE
        CALL SUBDE                      ;DIVIDE BY SUBTRACT
        JR NC,DV2                       ;AND COUNT
        ADD HL,DE
        RET
SUBDE:
        LD A,L                          ;*** SUBDE ***
        SUB E                           ;SUBSTRACT DE FROM
        LD L,A                          ;HL
        LD A,H
        SBC A,D
        LD H,A
        RET
CHKSGN:
        LD A,H                          ;*** CHKSGN ***
        OR A                            ;CHECK SIGN OF HL
        RET P
CHGSGN:
        LD A,H                          ;*** CHGSGN ***
        PUSH AF
        CPL                             ;CHANGE SIGN OF HL
        LD H,A
        LD A,L
        CPL
        LD L,A
        INC HL
        POP AF
        XOR H
        JP P,QHOW
        LD A,B                          ;AND ALSO FLIP B
        XOR 80H
        LD B,A
        RET
CKHLDE:
        LD A,H                          ;SAME SIGN?
        XOR D                           ;YES, COMPARE
        JP P,CK1                        ;NO, XCHANGE AND COMP
        EX DE,HL
CK1:
        JP RST20
;*************************************************************
;
; *** SETVAL *** FIN *** ENDCHK *** & ERROR (& FRIENDS) ***
;
; "SETVAL" EXPECTS A VARIABLE, FOLLOWED BY AN EQUAL SIGN AND
; THEN AN EXPR.  IT EVALUATES THE EXPR. AND SET THE VARIABLE
; TO THAT VALUE.
;
; "FIN" CHECKS THE END OF A COMMAND.  IF IT ENDED WITH ";",
; EXECUTION CONTINUES.  IF IT ENDED WITH A CR, IT FINDS THE
; NEXT LINE AND CONTINUE FROM THERE.
;
; "ENDCHK" CHECKS IF A COMMAND IS ENDED WITH CR.  THIS IS
; REQUIRED IN CERTAIN COMMANDS.  (GOTO, RETURN, AND STOP ETC.)
;
; "ERROR" PRINTS THE STRING POINTED BY DE (AND ENDS WITH CR).
; IT THEN PRINTS THE LINE POINTED BY 'CURRNT' WITH A "?"
; INSERTED AT WHERE THE OLD TEXT POINTER (SHOULD BE ON TOP
; OF THE STACK) POINTS TO.  EXECUTION OF TB IS STOPPED
; AND TBI IS RESTARTED.  HOWEVER, IF 'CURRNT' -> ZERO
; (INDICATING A DIRECT COMMAND), THE DIRECT COMMAND IS NOT
; PRINTED.  AND IF 'CURRNT' -> NEGATIVE # (INDICATING 'INPUT'
; COMMAND), THE INPUT LINE IS NOT PRINTED AND EXECUTION IS
; NOT TERMINATED BUT CONTINUED AT 'INPERR'.
;
; RELATED TO 'ERROR' ARE THE FOLLOWING:
; 'QWHAT' SAVES TEXT POINTER IN STACK AND GET MESSAGE "WHAT?"
; 'AWHAT' JUST GET MESSAGE "WHAT?" AND JUMP TO 'ERROR'.
; 'QSORRY' AND 'ASORRY' DO SAME KIND OF THING.
; 'AHOW' AND 'AHOW' IN THE ZERO PAGE SECTION ALSO DO THIS.
;*************************************************************

SETVAL:
        CALL RST38                         ;*** SETVAL ***
        JP C,QWHAT                      ;"WHAT?" NO VARIABLE
        PUSH HL                         ;SAVE ADDRESS OF VAR.
        CALL RST08                         ;PASS "=" SIGN
        DB '='
        DB SV1-$-1
        CALL RST18                         ;EVALUATE EXPR.
        LD B,H                          ;VALUE IS IN BC NOW
        LD C,L
        POP HL                          ;GET ADDRESS
        LD (HL),C                       ;SAVE VALUE
        INC HL
        LD (HL),B
        RET
SV1:
        JP QWHAT                        ;NO "=" SIGN
FIN:
        CALL RST08                         ;*** FIN ***
        DB ";"
        DB FI1-$-1
        POP AF                          ;";", PURGE RET. ADDR.
        JP RUNSML                       ;CONTINUE SAME LINE
FI1:
        CALL RST08                         ;NOT ";", IS IT CR?
        DB CR
        DB FI2-$-1
        POP AF                          ;YES, PURGE RET. ADDR.
        JP RUNNXL                       ;RUN NEXT LINE
FI2:
        RET                             ;ELSE RETURN TO CALLER
ENDCHK:
        CALL RST28                         ;*** ENDCHK ***
        CP CR                           ;END WITH CR?
        RET Z                           ;OK, ELSE SAY: "WHAT?"
QWHAT:
        PUSH DE                         ;*** QWHAT ***
AWHAT:
        LD DE,WHAT                      ;*** AWHAT ***
ERROR_ROUTINE:
        SUB A                           ;*** ERROR ***
        CALL PRTSTG                     ;PRINT 'WHAT?', 'HOW?'
        POP DE                          ;OR 'SORRY'
        LD A,(DE)                       ;SAVE THE CHARACTER
        PUSH AF                         ;AT WHERE OLD DE ->
        SUB A                           ;AND PUT A 0 THERE
        LD (DE),A
        LD HL,(CURRNT)                  ;GET CURRENT LINE #
        PUSH HL
        LD A,(HL)                       ;CHECK THE VALUE
        INC HL
        OR (HL)
        POP DE
        JP Z,RSTART                     ;IF ZERO, JUST RESTART
        LD A,(HL)                       ;IF NEGATIVE,
        OR A
        JP M,INPERR                     ;REDO INPUT
        CALL PRTLN                      ;ELSE PRINT THE LINE
        DEC DE                          ;UPTO WHERE THE 0 IS
        POP AF                          ;RESTORE THE CHARACTER
        LD (DE),A
        LD A,"?"                        ;PRINT A "?"
        CALL RST10
        SUB A                           ;AND THE REST OF THE
        CALL PRTSTG                     ;LINE
        JP RSTART                       ;THEN RESTART
QSORRY:
        PUSH DE                         ;*** QSORRY ***
ASORRY:
        LD DE,SORRY                     ;*** ASORRY ***
        JR ERROR_ROUTINE
;*************************************************************
;
; *** GETLN *** FNDLN (& FRIENDS) ***
;
; 'GETLN' READS A INPUT LINE INTO 'BUFFER'.  IT FIRST PROMPT
; THE CHARACTER IN A (GIVEN BY THE CALLER), THEN IT FILLS
; THE BUFFER AND ECHOS.  IT IGNORES LF'S AND NULLS, BUT STILL
; ECHOS THEM BACK.  BACKSPACE IS USED TO CAUSE IT TO DELETE
; THE LAST CHARACTER (IF THERE IS ONE), AND CTRL-Z IS USED TO
; CAUSE IT TO DELETE THE WHOLE LINE AND START IT ALL OVER.
; CR SIGNALS THE END OF A LINE, AND CAUSE 'GETLN' TO RETURN.
;
; 'FNDLN' FINDS A LINE WITH A GIVEN LINE # (IN HL) IN THE
; TEXT SAVE AREA.  DE IS USED AS THE TEXT POINTER.  IF THE
; LINE IS FOUND, DE WILL POINT TO THE BEGINNING OF THAT LINE
; (I.E., THE LOW BYTE OF THE LINE #), AND FLAGS ARE NC & Z.
; IF THAT LINE IS NOT THERE AND A LINE WITH A HIGHER LINE #
; IS FOUND, DE POINTS TO THERE AND FLAGS ARE NC & NZ.  IF
; WE REACHED THE END OF TEXT SAVE AREA AND CANNOT FIND THE
; LINE, FLAGS ARE C & NZ.
; 'FNDLN' WILL INITIALIZE DE TO THE BEGINNING OF THE TEXT SAVE
; AREA TO START THE SEARCH.  SOME OTHER ENTRIES OF THIS
; ROUTINE WILL NOT INITIALIZE DE AND DO THE SEARCH.
; 'FNDLNP' WILL START WITH DE AND SEARCH FOR THE LINE #.
; 'FNDNXT' WILL BUMP DE BY 2, FIND A CR AND THEN START SEARCH.
; 'FNDSKP' USE DE TO FIND A CR, AND THEN START SEARCH.
;*************************************************************

GETLN:
        CALL RST10                         ;*** GETLN ***
        LD DE,BUFFER                    ;PROMPT AND INIT.
GL1:
        CALL CHKIO                      ;CHECK KEYBOARD
        JR Z,GL1                        ;NO INPUT, WAIT
        CP BKSP                         ;DELETE LAST CHARACTER?
        JR Z,GL3                        ;YES
        CALL RST10                      ;INPUT, ECHO BACK
        CP LF                           ;IGNORE LF
        JR Z,GL1
        OR A                            ;IGNORE NULL
        JR Z,GL1
        CP CTRLZ                        ;DELETE THE WHOLE LINE?
        JR Z,GL4                        ;YES
        LD (DE),A                       ;ELSE SAVE INPUT
        INC DE                          ;AND BUMP POINTER
        CP CR                           ;WAS IT CR
        RET Z                           ;YES, END OF LINE
        LD A,E                          ;ELSE MORE FREE ROOM?
        CP BUFEND & 0FFH
        JR NZ,GL1                       ;YES, GET NEXT INPUT
GL3:
        LD A,E                          ;DELETE LAST CHARACTER
        CP BUFFER & 0FFH                ;BUT DO WE HAVE ANY?
        JR Z,GL4                        ;NO, REDO WHOLE LINE
        DEC DE                          ;YES, BACKUP POINTER
        LD A,5CH                        ;AND ECHO A BACK-SLASH
        CALL RST10
        JR GL1                          ;GO GET NEXT INPUT
GL4:
        CALL CRLF                       ;REDO ENTIRE LINE
        LD A,VT                         ;CR, LF AND UP-ARROW
        CALL RST10
        LD A,">"
        JR GETLN
FNDLN:
        LD A,H                          ;*** FNDLN ***
        OR A                            ;CHECK SIGN OF HL
        JP M,QHOW                       ;IT CANNOT BE -
        LD DE,TXTBGN                    ;INIT TEXT POINTER
FNDLP:                                  ;*** FDLNP ***
FL1:
        PUSH HL                         ;SAVE LINE #
        LD HL,(TXTUNF)                  ;CHECK IF WE PASSED END
        DEC HL
        CALL RST20
        POP HL                          ;GET LINE # BACK
        RET C                           ;C,NZ PASSED END
        LD A,(DE)                       ;WE DID NOT, GET BYTE 1
        SUB L                           ;IS THIS THE LINE?
        LD B,A                          ;COMPARE LOW ORDER
        INC DE
        LD A,(DE)                       ;GET BYTE 2
        SBC A,H                         ;COMPARE HIGH ORDER
        JR C,FL2                        ;NO, NOT THERE YET
        DEC DE                          ;ELSE WE EITHER FOUND
        OR B                            ;IT, OR IT IS NOT THERE
        RET                             ;NC,Z;FOUND, NC,NZ:NO
FNDNXT:                                 ;*** FNDNXT ***
        INC DE                          ;FIND NEXT LINE
FL2:
        INC DE                          ;JUST PASSED BYTE 1 & 2
FNDSKP:
        LD A,(DE)                       ;*** FNDSKP ***
        CP CR                           ;TRY TO FIND CR
        JR NZ,FL2                       ;KEEP LOOKING
        INC DE                          ;FOUND CR, SKIP OVER
        JR FL1                          ;CHECK IF END OF TEXT
;*************************************************************
;
; *** PRTSTG *** QTSTG *** PRTNUM *** & PRTLN ***
;
; 'PRTSTG' PRINTS A STRING POINTED BY DE.  IT STOPS PRINTING
; AND RETURNS TO CALLER WHEN EITHER A CR IS PRINTED OR WHEN
; THE NEXT BYTE IS THE SAME AS WHAT WAS IN A (GIVEN BY THE
; CALLER).  OLD A IS STORED IN B, OLD B IS LOST.
;
; 'QTSTG' LOOKS FOR A UNDERSCORE, SINGLE QUOTE, OR DOUBLE
; QUOTE.  IF NONE OF THESE, RETURN TO CALLER.  IF UNDERSCORE,
; OUTPUT A CR WITHOUT A LF.  IF SINGLE OR DOUBLE QUOTE, PRINT
; THE STRING IN THE QUOTE AND DEMANDS A MATCHING UNQUOTE.
; AFTER THE PRINTING THE NEXT 2 BYTES OF THE CALLER IS SKIPPED
; OVER (USUALLY A JUMP INSTRUCTION.
;
; 'PRTNUM' PRINTS THE NUMBER IN HL.  LEADING BLANKS ARE ADDED
; IF NEEDED TO PAD THE NUMBER OF SPACES TO THE NUMBER IN C.
; HOWEVER, IF THE NUMBER OF DIGITS IS LARGER THAN THE # IN
; C, ALL DIGITS ARE PRINTED ANYWAY.  NEGATIVE SIGN IS ALSO
; PRINTED AND COUNTED IN, POSITIVE SIGN IS NOT.
;
; 'PRTLN' PRINTS A SAVED TEXT LINE WITH LINE # AND ALL.
;
; 'PRTCHR' PRINTS THE VALUE IN L AS AN ASCII CHARACTER.  THIS
; WILL PRINT NON ASCII CHARACTERS TOO.  IE: ^G (BEL) 
;
; 'PRTASC' PRINTS THE VALUE IN L AS AN ASCII 
;
; 'PRTHEX' PRINTS THE VALUE IN HL AS A HEXIDECIMAL NUMBER. 
; HL IS TWO BYTES, SO TWO HEX VALUES WILL BE DISPLAYED. 
; IF H IS ZERO THEN ONLY ONE BYTE WILL BE DISPLAYED.
;
;*************************************************************

PRTSTG:
        LD B,A                          ;*** PRTSTG ***
PS1:
        LD A,(DE)                       ;GET A CHARACTER
        INC DE                          ;BUMP POINTER
        CP B                            ;SAME AS OLD A?
        RET Z                           ;YES, RETURN
        CALL RST10                         ;NO, NEXT
        CP CR                           ;WAS IT A CR?
        JR NZ,PS1                       ;NO, NEXT
        RET                             ;YES, RETURN
QTSTG:
        CALL RST08                         ;*** QTSTG ***
        DB 22H
        DB QT3-$-1
        LD A,22H                        ;IT IS A "
QT1:
        CALL PRTSTG                     ;PRINT UNTIL ANOTHER
        CP CR                           ;WAS LAST ONE A CR?
        POP HL                          ;RETURN ADDRESS
        JP Z,RUNNXL                     ;WAS CR, RUN NEXT LINE
QT2:
        INC HL                          ;SKIP 2 BYTES ON RETURN
        INC HL
        JP (HL)                         ;RETURN
QT3:
        CALL RST08                         ;IS IT A '?
        DB 27H
        DB QT4-$-1
        LD A,27H                        ;YES, DO THE SAME
        JR QT1                          ;AS IN "
QT4:
        CALL RST08                         ;IS IT UNDERSCORE?
        DB "_"
        DB QT5-$-1
        LD A,CR                            ;YES, CR WITHOUT LF
        CALL RST10                         ;DO IT TWICE TO GIVE
        LD A,VT                            ;YES, CR WITHOUT LF
        CALL RST10                         ;TTY ENOUGH TIME
        POP HL                          ;RETURN ADDRESS
        JR QT2
QT5:
        RET                             ;NONE OF ABOVE
;
PRTNUM:
        LD B,00H                        ;*** PRTNUM ***
        CALL CHKSGN                     ;CHECK SIGN
        JP P,PN1                        ;NO SIGN
        LD B,'-'                        ;B=SIGN
        DEC C                           ;'-' TAKES SPACE
PN1:
        PUSH DE                         ;SAVE
        LD DE,000AH                     ;DECIMAL
        PUSH DE                         ;SAVE AS FLAG
        DEC C                           ;C=SPACES
        PUSH BC                         ;SAVE SIGN & SPACE
PN2:
        CALL DIVIDE                     ;DIVIDE HL BY 10
        LD A,B                          ;RESULT 0?
        OR C
        JR Z,PN3                        ;YES, WE GOT ALL
        EX (SP),HL                      ;NO, SAVE REMAINDER
        DEC L                           ;AND COUNT SPACE
        PUSH HL                         ;HL IS OLD BC
        LD H,B                          ;MOVE RESULT TO BC
        LD L,C
        JR PN2                          ;AND DIVIDE BY 10
PN3:
        POP BC                          ;WE GOT ALL DIGITS IN
PN4:
        DEC C                           ;THE STACK
        LD A,C                          ;LOOK AT SPACE COUNT
        OR A
        JP M,PN5                        ;NO LEADING BLANKS
        LD A,20H                        ;LEADING BLANKS
        CALL RST10
        JR PN4                          ;MORE?
PN5:
        LD A,B                          ;PRINT SIGN
        OR A
        CALL NZ,RST10
        LD E,L                          ;LAST REMAINDER IN E
PN6:
        LD A,E                          ;CHECK DIGIT IN E
        CP 0AH                          ;10 IS FLAG FOR NO MORE
        POP DE
        RET Z                           ;IF SO, RETURN
        ADD A,30H                       ;ELSE, CONVERT TO ASCII
        CALL RST10                         ;PRINT THE DIGIT
        JR PN6                          ;GO BACK FOR MORE
PRTLN:
        LD A,(DE)                       ;*** PRTLN ***
        LD L,A                          ;LOW ORDER LINE #
        INC DE
        LD A,(DE)                       ;HIGH ORDER
        LD H,A
        INC DE
        LD C,04H                        ;PRINT 4 DIGIT LINE #
        CALL PRTNUM
        LD A,20H                        ;FOLLOWED BY A BLANK
        CALL RST10
        SUB A                           ;AND THEN THE NEXT
        JP PRTSTG
PRTCHR:
        LD A,L                          ;GET THE NUMBER
        AND DEL                         ;ONLY LOOK AT 0-128
PC1:
        JP RST10                        ;PRINT VALUE IN A
PRTASC:
        LD A,L                          ;GET THE NUMBER
        CP SPACE                        ;IS IT LOWER THAN ' ' ?
        JR C,PA1                        ;YES
        CP DEL                          ;IS IT HIGHER THAN '~' ?
        JR C,PC1                        ;NO
PA1:
        LD A,"."                        ;DISPLAY A . IF NON PRINTABLE
        JR PC1
PRTHEX:
        LD A,H
        OR A
        JR Z,PH1
        CALL A2HEX
PH1:
        LD A,L
A2HEX:
        PUSH AF                         ;SAVE A FOR SECOND NIBBLE
        RRCA                            ;SHIFT HIGH NIBBLE ACROSS
        RRCA                            ;
        RRCA                            ;
        RRCA                            ;
        CALL PH2                        ;CALL NIBBLE CONVERTER
        POP AF                          ;RECOVER LOW NIBBLE
PH2:
        AND 0FH                         ;MASK OFF HIGH NIBBLE
        ADD A,90H                       ;CONVERT TO
        DAA                             ;ASCII
        ADC A,40H                       ;USING THIS
        DAA                             ;AMAZING ROUTINE
        JP RST10                        ;DONE

;*************************************************************
;
; *** MVUP *** MVDOWN *** POPA *** & PUSHA ***
;
; 'MVUP' MOVES A BLOCK UP FROM WHERE DE-> TO WHERE BC-> UNTIL
; DE = HL
;
; 'MVDOWN' MOVES A BLOCK DOWN FROM WHERE DE-> TO WHERE HL->
; UNTIL DE = BC
;
; 'POPA' RESTORES THE 'FOR' LOOP VARIABLE SAVE AREA FROM THE
; STACK
;
; 'PUSHA' STACKS THE 'FOR' LOOP VARIABLE SAVE AREA INTO THE
; STACK
;*************************************************************

MVUP:
        CALL RST20                         ;*** MVUP ***
        RET Z                           ;DE = HL, RETURN
        LD A,(DE)                       ;GET ONE BYTE
        LD (BC),A                       ;MOVE IT
        INC DE                          ;INCREASE BOTH POINTERS
        INC BC
        JR MVUP                         ;UNTIL DONE
MVDOWN:
        LD A,B                          ;*** MVDOWN ***
        SUB D                           ;TEST IF DE = BC
        JP NZ,MD1                       ;NO, GO MOVE
        LD A,C                          ;MAYBE, OTHER BYTE?
        SUB E
        RET Z                           ;YES, RETURN
MD1:
        DEC DE                          ;ELSE MOVE A BYTE
        DEC HL                          ;BUT FIRST DECREASE
        LD A,(DE)                       ;BOTH POINTERS AND
        LD (HL),A                       ;THEN DO IT
        JR MVDOWN                       ;LOOP BACK
POPA:
        POP BC                          ;BC = RETURN ADDR.
        POP HL                          ;RESTORE LOPVAR, BUT
        LD (LOPVAR),HL                  ;=0 MEANS NO MORE
        LD A,H
        OR L
        JR Z,PP1                        ;YEP, GO RETURN
        POP HL                          ;NOP, RESTORE OTHERS
        LD (LOPINC),HL
        POP HL
        LD (LOPLMT),HL
        POP HL
        LD (LOPLN),HL
        POP HL
        LD (LOPPT),HL
PP1:
        PUSH BC                         ;BC = RETURN ADDR.
        RET
PUSHA:
        LD HL,STKLMT                    ;*** PUSHA ***
        CALL CHGSGN
        POP BC                          ;BC=RETURN ADDRESS
        ADD HL,SP                       ;IS STACK NEAR THE TOP?
        JP NC,QSORRY                    ;YES, SORRY FOR THAT
        LD HL,(LOPVAR)                  ;ELSE SAVE LOOP VAR'S
        LD A,H                          ;BUT IF LOPVAR IS 0
        OR L                            ;THAT WILL BE ALL
        JR Z,PU1
        LD HL,(LOPPT)                   ;ELSE, MORE TO SAVE
        PUSH HL
        LD HL,(LOPLN)
        PUSH HL
        LD HL,(LOPLMT)
        PUSH HL
        LD HL,(LOPINC)
        PUSH HL
        LD HL,(LOPVAR)
PU1:
        PUSH HL
        PUSH BC                         ;BC = RETURN ADDR.
        RET
;*************************************************************
;
; *** OUTC *** OUTP *** XON *** XOFF *** CHKIO ***
;
; THESE ARE THE ONLY I/O ROUTINES IN TBI.
; 'OUTC' IS CONTROLLED BY A SOFTWARE SWITCH 'OCSW'.  IF OCSW=0
; 'OUTC' WILL JUST RETURN TO THE CALLER.  IF OCSW IS NOT 0,
; IT WILL OUTPUT THE BYTE IN A.  IF THAT IS A CR, A LF IS ALSO
; SEND OUT.  ONLY THE FLAGS MAY BE CHANGED AT RETURN. ALL REG.
; ARE RESTORED.
;
; 'OUTP' LETS THE USER SEND DATA TO AN OUTPUT PORT ON THE
; CONTROLING HARDWARE.  THE FORMAT IS 'OUT <PORT>,<DATA>'
; WHERE <PORT> IS THE PORT NUMBER AND <DATA> IS THE DATA
; TO OUTPUT.  BOTH PORT NUMBER AND DATA ARE EXPRESIONS AND
; ARE TO BE LESS THAN 256.  
;
; 'XON' AND 'XOFF' PROGRAMATICALLY TURN ON OR OFF THE 
; OUTPUT SWITCH.  THIS IS THE SAME AS PRESSING CONTROL-O
; ON THE KEYBOARD.  THE CAN BE USED TO DISABLE TX_RDY,
; THE SERIAL OUTPUT ROUTINE, IF THE SAME PORT IS BEING 
; USED.
;
; 'CHKIO' CHECKS THE INPUT.  IF NO INPUT, IT WILL RETURN TO
; THE CALLER WITH THE Z FLAG SET.  IF THERE IS INPUT, Z FLAG
; IS CLEARED AND THE INPUT BYTE IS IN A.  HOWEVER, IF THE
; INPUT IS A CONTROL-O, THE 'OCSW' SWITCH IS COMPLIMENTED, AND
; Z FLAG IS RETURNED.  IF A CONTROL-C IS READ, 'CHKIO' WILL
; RESTART TBI AND DO NOT RETURN TO THE CALLER.
;
; Do not modify these routines.  Routines requiring
; modification are : SERIAL_INIT, RX_RDY, and TX_RDY.
;*************************************************************

INIT:
        CALL SERIAL_INIT        ;INITIALIZE THE SIO
        XOR A
        LD DE,MSG1          ;PRINT THE BOOT MESSAGES
        CALL PRTSTG
        XOR A
        LD DE,MSG2
        CALL PRTSTG
        LD HL,START
        LD (RANPNT),HL
        LD HL,TXTBGN
        LD (TXTUNF),HL
        JP RSTART
OUTC:
        JR NZ,OUTC2                     ;IT IS ON
        POP AF                          ;IT IS OFF
        RET                             ;RESTORE AF AND RETURN
OUTC2:
        POP AF                          ;RESTORE THE REGISTER         
        CALL TX_RDY                     ;SEND THE BYTE
        CP CR
        RET NZ
        LD A,LF
        CALL RST10
        LD A,CR
        RET
OUTP:
        CALL RST18                      ;GET PORT NUMBER
        XOR A                           ;IS PORT > 255?
        CP H
        JP NZ,QHOW                      ;YES, NOT A VALID PORT
        CALL RST08                      ;IF "," THEN GET DATA
        DB ','
        DB OT1-$-1
        PUSH HL
        CALL RST18                      ;GET DATA TO OUTPUT
        XOR A                           ;IS DATA > 255?
        CP H
        JP NZ,QHOW                      ;YES, NOT A DATA VALUE
        LD A,L                          ;RETRIVE DATA
        POP HL
        LD C,L                          ;RETRIVE PORT
        OUT (C),A                       ;SEND IT OUT
        CALL RST30
OT1:
        JP QWHAT
XON:
        LD A,0FFH                       ;SET SWTICH TO ON
        JR XO1
XOFF:
        XOR A                           ;SET SWITCH TO OFF
XO1:
        LD (OCSW),A                   ;SAVE NEW SWITCH
        CALL RST30                      ;CONTINUE
CHKIO:
        CALL RX_RDY                     ;CHECK IF CHARACTER AVAILABLE
        RET Z                           ;RETURN IF NO CHARACTER AVAILABLE

        PUSH BC                         ;IF IT'S A LF, IGNORE AND RETURN
        LD B,A                          ; AS IF THERE WAS NO CHARACTER.
        SUB LF
        JR Z,CHKIO2
        LD A,B                          ;OTHERWISE RESTORE 'A' AND 'BC'
        POP BC                          ; AND CONTINUE ON.

        CP CTRLO                        ;IS IT CONTROL-0?
        JR NZ,CI1                       ;NO, MORE CHECKING
        LD A,(OCSW)                     ;CONTROL-0 FLIPS OCSW
        CPL                             ;ON TO OFF, OFF TO ON
        LD (OCSW),A
        JR CHKIO                        ;GET ANOTHER INPUT
CHKIO2:
        XOR A                           ;CLEAR A
        POP BC                          ;RESTORE THE 'BC' PAIR
        RET                             ;RETURN WITH 'Z' SET.

CI1:
        CP 61H                          ;IS IT LOWER THAN 'a' ?
        JR C,CI2                        ;YES
        CP 7BH                          ;IS IT HIGHT THAN 'z' ?
        JR NC,CI2                       ;YES
        AND 0DFH                        ;MAKE IT UPPER CASE
CI2:
        CP CTRLC                        ;IS IT CONTROL-C?
        JP Z,RSTART                     ;YES, RESTART TBI
        CP CTRLD                        ;IS IT CONTROL-D?
        RET NZ                          ;NO, RETURN "NZ"
        RST 00H                         ;RESTART MONITOR AND EXIT


MSG1:   DB   CS,'Z80 TINY BASIC 2.1b',CR       ;CLEAR SCREEN AND BOOT MESSAGE
MSG2:   DB   'TEC-1F VERSION BY B CHIHA, 2022',CR

;*************************************************************
;
; *** TABLES *** DIRECT *** & EXEC ***
;
; THIS SECTION OF THE CODE TESTS A STRING AGAINST A TABLE.
; WHEN A MATCH IS FOUND, CONTROL IS TRANSFERED TO THE SECTION
; OF CODE ACCORDING TO THE TABLE.
;
; AT 'EXEC', DE SHOULD POINT TO THE STRING AND HL SHOULD POINT
; TO THE TABLE-1.  AT 'DIRECT', DE SHOULD POINT TO THE STRING.
; HL WILL BE SET UP TO POINT TO TAB1-1, WHICH IS THE TABLE OF
; ALL DIRECT AND STATEMENT COMMANDS.
;
; A '.' IN THE STRING WILL TERMINATE THE TEST AND THE PARTIAL
; MATCH WILL BE CONSIDERED AS A MATCH.  E.G., 'P.', 'PR.',
; 'PRI.', 'PRIN.', OR 'PRINT' WILL ALL MATCH 'PRINT'.
;
; THE TABLE CONSISTS OF ANY NUMBER OF ITEMS.  EACH ITEM
; IS A STRING OF CHARACTERS WITH BIT 7 SET TO 0 AND
; A JUMP ADDRESS STORED HI-LOW WITH BIT 7 OF THE HIGH
; BYTE SET TO 1.
;
; END OF TABLE IS AN ITEM WITH A JUMP ADDRESS ONLY.  IF THE
; STRING DOES NOT MATCH ANY OF THE OTHER ITEMS, IT WILL
; MATCH THIS NULL ITEM AS DEFAULT.
;*************************************************************

TAB1:                                   ;DIRECT COMMANDS
        DB 'LIST'
        DWA LIST
        DB 'RUN'
        DWA RUN
        DB 'NEW'
        DWA NEW
TAB2:                                   ;DIRECT/STATEMENT
        DB 'NEXT'
        DWA NEXT
        DB 'LET'
        DWA LET
        DB 'IF'
        DWA IFF
        DB 'GOTO'
        DWA GOTO
        DB 'GOSUB'
        DWA GOSUB
        DB 'RETURN'
        DWA RETURN
        DB 'REM'
        DWA REM
        DB 'FOR'
        DWA FOR
        DB 'INPUT'
        DWA INPUT
        DB 'PRINT'
        DWA PRINT
        DB 'OUT'
        DWA OUTP
        DB 'STOP'
        DWA STOP
        DB 'XON'
        DWA XON
        DB 'XOFF'
        DWA XOFF
        DWA DEFLT
TAB4:                                   ;FUNCTIONS
        DB 'RND'
        DWA RND
        DB 'ABS'
        DWA ABS
        DB 'PEEK'
        DWA PEEK
        DB 'SIZE'
        DWA SIZE
        DWA XP40
TAB5:                                   ;"TO" IN "FOR"
        DB 'TO'
        DWA FR1
        DWA QWHAT
TAB6:                                   ;"STEP" IN "FOR"
        DB 'STEP'
        DWA FR2
        DWA FR3
TAB8:                                   ;RELATION OPERATORS
        DB '>='
        DWA XP11
        DB '#'
        DWA XP12
        DB '>'
        DWA XP13
        DB '='
        DWA XP15
        DB '<='
        DWA XP14
        DB '<'
        DWA XP16
        DWA XP17
DIRECT: LD HL,TAB1-1                   ;*** DIRECT ***
EXEC:                                   ;*** EXEC ***
EX0:    CALL RST28                         ;IGNORE LEADING BLANKS
        PUSH DE                         ;SAVE POINTER
EX1:
        LD A,(DE)                       ;IF FOUND '.' IN STRING
        INC DE                          ;BEFORE ANY MISMATCH
        CP "."                          ;WE DECLARE A MATCH
        JR Z,EX3
        INC HL                          ;HL->TABLE
        CP (HL)                         ;IF MATCH, TEST NEXT
        JR Z,EX1
        LD A,7FH                        ;ELSE SEE IF BIT 7
        DEC DE                          ;OF TABLE IS SET, WHICH
        CP (HL)                         ;IS THE JUMP ADDR. (HI)
        JR C,EX5                        ;C:YES, MATCHED
EX2:
        INC HL                          ;NC:NO, FIND JUMP ADDR.
        CP (HL)
        JR NC,EX2
        INC HL                          ;BUMP TO NEXT TAB. ITEM
        POP DE                          ;RESTORE STRING POINTER
        JR EX0                          ;TEST AGAINST NEXT ITEM
EX3:
        LD A,7FH                        ;PARTIAL MATCH, FIND
EX4:
        INC HL                          ;JUMP ADDR., WHICH IS
        CP (HL)                         ;FLAGGED BY BIT 7
        JR NC,EX4
EX5:
        LD A,(HL)                       ;LOAD HL WITH THE JUMP
        INC HL                          ;ADDRESS FROM THE TABLE
        LD L,(HL)
        AND 7FH                         ;MASK OFF BIT 7
        LD H,A
        POP AF                          ;CLEAN UP THE GABAGE
        JP (HL)                         ;AND WE GO DO IT
;-------------------------------------------------------------------------------
;///////////////////////////////////////////////////////////////////////////////
;-------------------------------------------------------------------------------
;COMPUTER SPECIFIC ROUTINES.
;-------------------------------------------------------------------------------
SERIAL_INIT:
        jp sioa_init                    ;JUMP THROUGH
;-------------------------------------------------------------------------------
TX_RDY:

    ; This routine sends the character to the output port.  Port is always
    ; ready as init and last part sets output port to high for two stop bits

    PUSH AF
    PUSH BC

    LD C,A                 ; sioa_tx_char expects char in C iso A
    CALL sioa_tx_char

    POP BC
    POP AF

    RET

;-------------------------------------------------------------------------------
RX_RDY:

    ; This routine is for checkif if a character is available over
    ; serial. If a character is available, it returns to the calling
    ; function with the character in 'A' and the Z-flag reset.
    ; However, if a character is not available, it returns with the
    ; Z-flag set.

    CALL sioa_rx_ready
    RET Z               ; zero flag is set

    PUSH BC
    CALL sioa_rx_char

    POP BC
    ; CHECK IF ZERO FLAG NEEDS TO BE CLEARED
    RET

    DS 07H                          ;FILL TO MAKE EVEN AF NEEDED
;-------------------------------------------------------------------------------

;///////////////////////////////////////////////////////////////////////////////
;-------------------------------------------------------------------------------

include '../lib/sio.asm'


LSTROM:                                 ;ALL ABOVE CAN BE ROM
                    ;HERE DOWN MUST BE RAM
        END


