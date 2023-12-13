#!/bin/sh

# Copyright (c) 2023, LexxPluss Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

cansend can0 000#0101
cansend can0 000#0102

# id 1, RPDO 1
cansend can0 601#23.0014.01.0102.0080 # cob-id 201
cansend can0 601#2F.0014.02.FF.000000 # transmission mode
cansend can0 601#2F.0016.00.00000000  # disconnect PDO 1600
cansend can0 601#23.0016.01.10004060  # 6040
cansend can0 601#2F.0016.00.01.000000 # start PDO 1600
cansend can0 601#23.0014.01.0102.0000 # start PDO 1400
# id 2, RPDO 1
cansend can0 602#23.0014.01.0202.0080 # cob-id 202
cansend can0 602#2F.0014.02.FF.000000 # transmission mode
cansend can0 602#2F.0016.00.00000000  # disconnect PDO 1600
cansend can0 602#23.0016.01.10004060  # 6040
cansend can0 602#2F.0016.00.01.000000 # start PDO 1600
cansend can0 602#23.0014.01.0202.0000 # start PDO 1400

# id 1, RPDO 2
cansend can0 601#23.0114.01.0103.0080 # cob-id 301
cansend can0 601#2F.0114.02.FF.000000 # transmission mode
cansend can0 601#2F.0116.00.00000000  # disconnect PDO 1601
cansend can0 601#23.0116.01.08006060  # 6060
cansend can0 601#2F.0116.00.01.000000 # start PDO 1601
cansend can0 601#23.0114.01.0103.0000 # start PDO 1401
# id 2, RPDO 2
cansend can0 602#23.0114.01.0203.0080 # cob-id 302
cansend can0 602#2F.0114.02.FF.000000 # transmission mode
cansend can0 602#2F.0116.00.00000000  # disconnect PDO 1601
cansend can0 602#23.0116.01.08006060  # 6060
cansend can0 602#2F.0116.00.01.000000 # start PDO 1601
cansend can0 602#23.0114.01.0203.0000 # start PDO 1401

# id 1, RPDO 3
cansend can0 601#23.0214.01.0104.0080 # cob-id 401
cansend can0 601#2F.0214.02.FF.000000 # transmission mode
cansend can0 601#2F.0216.00.00000000  # disconnect PDO 1602
cansend can0 601#23.0216.01.20007A60  # 607A
cansend can0 601#23.0216.02.10007160  # 6071
cansend can0 601#2F.0216.00.02.000000 # start PDO 1602
cansend can0 601#23.0214.01.0104.0000 # start PDO 1402
# id 2, RPDO 3
cansend can0 602#23.0214.01.0204.0080 # cob-id 402
cansend can0 602#2F.0214.02.FF.000000 # transmission mode
cansend can0 602#2F.0216.00.00000000  # disconnect PDO 1602
cansend can0 602#23.0216.01.20007A60  # 607A
cansend can0 602#23.0216.02.10007160  # 6071
cansend can0 602#2F.0216.00.02.000000 # start PDO 1602
cansend can0 602#23.0214.01.0204.0000 # start PDO 1402

# id 1, RPDO 4
cansend can0 601#23.0314.01.0105.0080 # cob-id 501
cansend can0 601#2F.0314.02.FF.000000 # transmission mode
cansend can0 601#2F.0316.00.00000000  # disconnect PDO 1603
cansend can0 601#23.0316.01.2000FF60  # 60FF
cansend can0 601#23.0316.02.20008160  # 6081
cansend can0 601#2F.0316.00.02.000000 # start PDO 1603
cansend can0 601#23.0314.01.0105.0000 # start PDO 1403
# id 2, RPDO 4
cansend can0 602#23.0314.01.0205.0080 # cob-id 502
cansend can0 602#2F.0314.02.FF.000000 # transmission mode
cansend can0 602#2F.0316.00.00000000  # disconnect PDO 1603
cansend can0 602#23.0316.01.2000FF60  # 60FF
cansend can0 602#23.0316.02.20008160  # 6081
cansend can0 602#2F.0316.00.02.000000 # start PDO 1603
cansend can0 602#23.0314.01.0205.0000 # start PDO 1403

sleep 1
# id 1, RPDO 5
cansend can0 601#23.0414.01.00000080  # cob-id -
cansend can0 601#2F.0414.02.FF.000000 # transmission mode
cansend can0 601#2F.0416.00.00000000  # disconnect PDO 1604
# id 2, RPDO 5
cansend can0 602#23.0414.01.00000080  # cob-id -
cansend can0 602#2F.0414.02.FF.000000 # transmission mode
cansend can0 602#2F.0416.00.00000000  # disconnect PDO 1604
# id 1, RPDO 6
cansend can0 601#23.0514.01.00000080  # cob-id -
cansend can0 601#2F.0514.02.FF.000000 # transmission mode
cansend can0 601#2F.0516.00.00000000  # disconnect PDO 1605
# id 2, RPDO 6
cansend can0 602#23.0514.01.00000080  # cob-id -
cansend can0 602#2F.0514.02.FF.000000 # transmission mode
cansend can0 602#2F.0516.00.00000000  # disconnect PDO 1605
# id 1, RPDO 7
cansend can0 601#23.0614.01.00000080  # cob-id -
cansend can0 601#2F.0614.02.FF.000000 # transmission mode
cansend can0 601#2F.0616.00.00000000  # disconnect PDO 1606
# id 2, RPDO 7
cansend can0 602#23.0614.01.00000080  # cob-id -
cansend can0 602#2F.0614.02.FF.000000 # transmission mode
cansend can0 602#2F.0616.00.00000000  # disconnect PDO 1606
# id 1, RPDO 8
cansend can0 601#23.0714.01.00000080  # cob-id -
cansend can0 601#2F.0714.02.FF.000000 # transmission mode
cansend can0 601#2F.0716.00.00000000  # disconnect PDO 1607
# id 2, RPDO 8
cansend can0 602#23.0714.01.00000080  # cob-id -
cansend can0 602#2F.0714.02.FF.000000 # transmission mode
cansend can0 602#2F.0716.00.00000000  # disconnect PDO 1607

sleep 1

# id 1, TPDO 1
cansend can0 601#2F.001A.0000000000 # disconnect pdo 1A00
cansend can0 601#23.001A.01.10004160 # 6041
cansend can0 601#23.001A.02.20006460 # 6064
cansend can0 601#23.0018.01.8101.0000 # 0181
cansend can0 601#2F.0018.02.FF.000000 # sync
cansend can0 601#2F.001A.00.02.000000 # enable 1A00
# id 2, TPDO 1
cansend can0 602#2F.001A.0000000000 # disconnect pdo 1A00
cansend can0 602#23.001A.01.10004160 # 6041
cansend can0 602#23.001A.02.20006460 # 6064
cansend can0 602#23.0018.01.8201.0000 # 0182
cansend can0 602#2F.0018.02.FF.000000 # sync
cansend can0 602#2F.001A.00.02.000000 # enable 1A00

# id 1, TPDO 2
cansend can0 601#2F.011A.0000000000 # disconnect pdo 1A01
cansend can0 601#23.011A.01.08006160 # 6161
cansend can0 601#23.0118.01.8102.0000 # 0281
cansend can0 601#2F.0118.02.FF.000000 # sync
cansend can0 601#2F.011A.00.01.000000 # enable 1A01
# id 2, TPDO 2
cansend can0 602#2F.011A.0000000000 # disconnect pdo 1A01
cansend can0 602#23.011A.01.08006160 # 6161
cansend can0 602#23.0118.01.8202.0000 # 0282
cansend can0 602#2F.0118.02.FF.000000 # sync
cansend can0 602#2F.011A.00.01.000000 # enable 1A01

# id 1, TPDO 3
cansend can0 601#2F.021A.0000000000 # disconnect pdo 1A02
cansend can0 601#23.021A.01.20006960 # 6069
cansend can0 601#23.0218.01.8103.0000 # 0381
cansend can0 601#2F.0218.02.FF.000000 # sync
cansend can0 601#2F.021A.00.01.000000 # enable 1A02
# id 2, TPDO 3
cansend can0 602#2F.021A.0000000000 # disconnect pdo 1A02
cansend can0 602#23.021A.01.20006960 # 6069
cansend can0 602#23.0218.01.8203.0000 # 0382
cansend can0 602#2F.0218.02.FF.000000 # sync
cansend can0 602#2F.021A.00.01.000000 # enable 1A02

# id 1, TPDO 4
cansend can0 601#2F.031A.0000000000 # disconnect pdo 1A03
cansend can0 601#23.031A.01.20006C60 # 606C
cansend can0 601#23.031A.02.10007760 # 6077
cansend can0 601#23.0318.01.8104.0000 # 0481
cansend can0 601#2F.0318.02.FF.000000 # sync
cansend can0 601#2F.031A.00.02.000000 # enable 1A03
# id 2, TPDO 4
cansend can0 602#2F.031A.0000000000 # disconnect pdo 1A03
cansend can0 602#23.031A.01.20006C60 # 606C
cansend can0 602#23.031A.02.10007760 # 6077
cansend can0 602#23.0318.01.8204.0000 # 0482
cansend can0 602#2F.0318.02.FF.000000 # sync
cansend can0 602#2F.031A.00.02.000000 # enable 1A03

# id 1, TPDO 5
cansend can0 601#2F.041A.0000000000 # disconnect pdo 1A04
cansend can0 601#23.0418.01.00000080
cansend can0 601#2F.0418.02.00.000000 # sync
# id 2, TPDO 5
cansend can0 602#2F.041A.0000000000 # disconnect pdo 1A04
cansend can0 602#23.0418.01.00000080
cansend can0 602#2F.0418.02.00.000000 # sync
# id 1, TPDO 6
cansend can0 601#2F.051A.0000000000 # disconnect pdo 1A05
cansend can0 601#23.0518.01.00000080
cansend can0 601#2F.0518.02.00.000000 # sync
# id 2, TPDO 6
cansend can0 602#2F.051A.0000000000 # disconnect pdo 1A05
cansend can0 602#23.0518.01.00000080
cansend can0 602#2F.0518.02.00.000000 # sync
# id 1, TPDO 7
cansend can0 601#2F.061A.0000000000 # disconnect pdo 1A06
cansend can0 601#23.0618.01.00000080
cansend can0 601#2F.0618.02.00.000000 # sync
# id 2, TPDO 7
cansend can0 602#2F.061A.0000000000 # disconnect pdo 1A06
cansend can0 602#23.0618.01.00000080
cansend can0 602#2F.0618.02.00.000000 # sync
# id 1, TPDO 8
cansend can0 601#2F.071A.0000000000 # disconnect pdo 1A07
cansend can0 601#23.0718.01.00000080
cansend can0 601#2F.0718.02.00.000000 # sync
# id 1, TPDO 8
cansend can0 602#2F.071A.0000000000 # disconnect pdo 1A07
cansend can0 602#23.0718.01.00000080
cansend can0 602#2F.0718.02.00.000000 # sync


sleep 1

# save
cansend can0 601#23.2024.00.0A000000
cansend can0 601#23.1010.01.73617665
cansend can0 602#23.2024.00.0A000000
cansend can0 602#23.1010.01.73617665

sleep 1
