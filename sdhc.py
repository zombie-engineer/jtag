
RESP_TYPE_R1  = 1
RESP_TYPE_R1b = 2
RESP_TYPE_R2  = 3
RESP_TYPE_R3  = 4
RESP_TYPE_R6  = 5
RESP_TYPE_R7  = 6
RESP_TYPE_NA  = 7

DIR_UNKNOWN   = 0
DIR_HOST2CARD = 0
DIR_CARD2HOST = 1


# Entry format CMD_ID : (RESP_TYPE, CHECK_CRC, DIR, HAS_DATA, MULTIBLOCK)
sd_commands = {
   0 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
   1 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
   2 : (RESP_TYPE_R2,   0, DIR_UNKNOWN  , 0, 0),
   3 : (RESP_TYPE_R6,   1, DIR_UNKNOWN  , 0, 0),
   4 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
   5 : (RESP_TYPE_R1b,  0, DIR_UNKNOWN  , 0, 0),
   6 : (RESP_TYPE_R1,   1, DIR_CARD2HOST, 1, 0),
   7 : (RESP_TYPE_R1b,  1, DIR_UNKNOWN  , 0, 0),
   8 : (RESP_TYPE_R7,   1, DIR_UNKNOWN  , 0, 0),
   9 : (RESP_TYPE_R2,   0, DIR_UNKNOWN  , 0, 0),
  10 : (RESP_TYPE_R2,   0, DIR_UNKNOWN  , 0, 0),
  11 : (RESP_TYPE_R1,   1, DIR_UNKNOWN  , 0, 0),
  12 : (RESP_TYPE_R1b,  1, DIR_UNKNOWN  , 0, 0),
  13 : (RESP_TYPE_R1,   1, DIR_UNKNOWN  , 0, 0),
  14 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  15 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  16 : (RESP_TYPE_R1,   1, DIR_UNKNOWN  , 0, 0),
  17 : (RESP_TYPE_R1,   1, DIR_CARD2HOST, 1, 0),
  18 : (RESP_TYPE_R1,   1, DIR_CARD2HOST, 1, 1),
  19 : (RESP_TYPE_R1,   1, DIR_UNKNOWN  , 0, 0),
  20 : (RESP_TYPE_R1b,  1, DIR_UNKNOWN  , 0, 0),
  21 : (RESP_TYPE_R1,   1, DIR_UNKNOWN  , 0, 0),
  22 : (RESP_TYPE_R1,   1, DIR_UNKNOWN  , 0, 0),
  23 : (RESP_TYPE_R1,   1, DIR_UNKNOWN  , 0, 0),
  24 : (RESP_TYPE_R1,   1, DIR_HOST2CARD, 1, 0),
  25 : (RESP_TYPE_R1,   1, DIR_HOST2CARD, 1, 1),
  26 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  27 : (RESP_TYPE_R1,   1, DIR_UNKNOWN  , 0, 0),
  28 : (RESP_TYPE_R1b,  1, DIR_UNKNOWN  , 0, 0),
  29 : (RESP_TYPE_R1b,  1, DIR_UNKNOWN  , 0, 0),
  30 : (RESP_TYPE_R1,   1, DIR_UNKNOWN  , 0, 0),
  31 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  32 : (RESP_TYPE_R1,   1, DIR_UNKNOWN  , 0, 0),
  33 : (RESP_TYPE_R1,   1, DIR_UNKNOWN  , 0, 0),
  34 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  35 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  36 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  37 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  38 : (RESP_TYPE_R1b,  1, DIR_UNKNOWN  , 0, 0),
  39 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  40 : (RESP_TYPE_R1,   1, DIR_UNKNOWN  , 0, 0),
  41 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  42 : (RESP_TYPE_R1,   1, DIR_UNKNOWN  , 0, 0),
  43 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  44 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  45 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  46 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  47 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  48 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  49 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  50 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  51 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  52 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  53 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  54 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  55 : (RESP_TYPE_R1,   1, DIR_UNKNOWN  , 0, 0),
  56 : (RESP_TYPE_R1,   1, DIR_UNKNOWN  , 0, 0),
  57 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  58 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  59 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0)
}

sd_acommands = {
   0 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
   1 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
   2 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
   3 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
   4 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
   5 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
   6 : (RESP_TYPE_R1,    1, DIR_UNKNOWN, 0, 0),
   7 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
   8 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
   9 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  10 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  11 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  12 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  13 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  14 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  15 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  16 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  17 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  18 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  19 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  20 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  21 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  22 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  23 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  24 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  25 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  26 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  27 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  28 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  29 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  30 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  31 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  32 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  33 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  34 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  35 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  36 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  37 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  38 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  39 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  40 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  41 : (RESP_TYPE_R3,    0, DIR_UNKNOWN, 0, 0),
  42 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  43 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  44 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  45 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  46 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  47 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  48 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  49 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  50 : (RESP_TYPE_R1,    1, DIR_UNKNOWN, 0, 0),
  51 : (RESP_TYPE_R1,    1, DIR_CARD2HOST, 1, 0),
  52 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  53 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  54 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  55 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  56 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  57 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  58 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  59 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0)
}


class cmd_result:
  def __init__(self, status, data, resp0, resp1, resp2, resp3):
    self.status = status
    self.data = data
    self.resp0 = resp0
    self.resp1 = resp1
    self.resp2 = resp2
    self.resp3 = resp3
