package org.usfirst.frc.team5414.robot.commands;

import java.util.ArrayList;
import java.util.StringTokenizer;

import org.usfirst.frc.team5414.robot.Robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoSwitchMtoL extends CommandGroup {

    public AutoSwitchMtoL() {
    	StringTokenizer st = null;
    	ArrayList<Double> left = new ArrayList<>();
    	ArrayList<Double> right = new ArrayList<>();
//    	String path = "0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 1 1 1 1 1 1 2 2 2 2 3 2 3 3 3 3 4 4 5 4 5 5 6 6 7 6 7 7 8 7 9 8 10 9 11 10 12 11 13 12 14 13 15 14 15 15 16 16 17 17 18 18 19 19 21 21 22 22 23 24 24 25 25 27 27 28 28 30 29 32 30 34 31 35 33 37 34 39 35 41 37 43 38 45 39 48 41 50 42 52 43 54 44 56 46 59 47 61 48 64 50 67 51 69 52 72 54 74 55 77 56 80 57 83 59 85 60 88 61 91 63 94 64 97 65 100 66 102 68 105 69 108 70 111 72 114 73 117 74 120 75 123 77 126 78 129 79 132 80 134 81 137 83 140 84 143 85 146 87 149 88 152 89 154 90 157 92 160 93 163 94 166 96 168 97 171 98 173 99 176 101 179 102 182 103 184 104 187 106 189 107 192 108 195 110 197 111 200 112 202 113 205 114 208 116 210 117 213 118 216 119 219 120 222 121 224 123 227 124 230 125 233 126 235 127 238 128 241 129 244 130 247 131 250 132 253 133 256 134 258 135 261 136 264 137 267 138 270 139 273 140 276 141 280 142 283 142 286 143 289 144 292 145 295 145 298 146 301 147 304 148 307 148 310 149 314 150 317 151 320 152 323 152 326 153 329 154 332 155 335 155 338 156 341 157 345 158 348 159 351 160 354 161 356 162 359 163 362 164 365 165 368 166 371 167 374 168 376 169 379 170 382 171 384 172 387 174 389 175 392 176 395 178 397 179 400 180 402 182 404 183 407 184 409 186 411 187 414 189 416 190 418 192 421 194 423 195 425 197 427 198 429 200 431 202 433 203 435 205 437 207 439 208 441 210 443 212 445 214 447 215 448 217 450 219 452 221 454 223 456 225 457 227 459 229 461 230 463 232 464 234 466 236 468 238 470 241 471 243 473 245 475 247 477 249 478 251 480 254 482 256 484 258 485 261 487 263 489 266 490 268 492 270 494 273 495 275 497 278 499 281 501 283 502 286 504 289 506 291 507 294 509 297 511 299 512 302 514 306 517 308 518 311 520 314 521 316 523 319 525 322 527 325 529 328 530 331 532 334 534 337 536 340 538 343 540 346 542 349 544 352 546 355 548 357 550 360 552 364 554 367 556 370 559 373 561 376 563 379 565 383 568 385 570 390 573 393 575 397 578 402 581 403 582 404 583 407 585 410 587 413 590 416 592 420 594 422 596 425 599 428 601 432 603 435 606 438 608 441 610 444 613 448 615 451 617 454 620 457 622 460 624 463 627 466 629 470 631 473 634 476 636 479 638 483 640 486 643 489 645 493 647 496 649 500 652 503 654 507 656 510 658 514 660 517 663 521 665 524 667 528 669 531 671 535 673 539 675 542 677 546 679 550 681 554 683 557 685 561 687 565 689 569 691 573 693 576 695 580 697 584 699 588 701 592 703 596 705 600 706 604 708 608 710 612 712 616 714 620 715 624 717 628 719 632 721 636 723 640 724 644 726 647 728 651 730 655 732 660 734 664 735 668 737 672 739 677 742 680 743 683 745 687 747 691 749 695 751 699 753 704 756 707 758 711 760 715 762 719 764 722 766 726 768 729 771 733 773 736 775 740 777 743 780 747 782 750 784 754 787 757 789 761 791 764 794 767 796 770 798 774 801 776 803 779 805 782 807 785 810 788 812 791 813 793 815 796 818 799 820 801 822 804 824 806 826 808 828 811 830 813 831 815 833 817 835 819 836 821 838 823 839 825 840 827 842 829 843 831 844 833 846 835 847 836 848 838 849 840 850 842 851 843 852 845 853 846 853 848 854 849 855 851 856 852 857 854 857 855 858 856 859 858 859 859 860 860 860 861 860 862 861 863 861 864 861 865 862 865 862 866 862 867 862 867 862";
    	String path = "0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 2 2 3 3 4 3 5 4 6 5 7 7 9 8 10 10 12 11 14 13 16 15 18 17 20 19 22 22 24 24 27 27 30 29 32 32 35 35 38 38 41 41 45 45 48 48 52 52 55 55 59 59 63 63 67 67 71 71 75 75 79 80 83 84 88 89 92 93 97 98 101 103 105 107 110 112 115 117 120 122 125 127 130 133 135 138 140 143 145 148 177 182 180 185 182 187 184 188 186 191 189 194 193 198 196 200 199 204 202 207 206 211 210 215 216 221 221 227 227 233 233 238 239 244 244 250 250 256 256 262 262 268 268 274 274 280 280 286 286 292 292 299 298 305 304 311 310 317 316 323 322 330 328 336 335 342 341 349 347 355 354 362 360 368 366 374 372 381 379 387 386 394 392 400 398 406 405 413 412 419 418 425 425 432 431 438 438 444 445 451 452 457 459 464 466 470 473 476 479 482 487 489 493 495 500 501 507 508 514 514 521 520 528 526 535 533 542 539 550 545 557 552 564 558 571 564 578 570 585 577 593 583 600 589 607 595 614 601 621 607 628 613 636 620 643 626 650 632 657 638 664 644 671 651 678 657 686 664 693 670 700 677 707 684 714 690 721 697 728 703 735 710 743 717 749 724 756 730 763 737 770 744 777 751 784 758 791 765 805 778 807 781 813 786 819 793 826 800 834 807 840 814 847 821 854 828 861 835 868 842 875 849 882 856 888 863 895 870 902 877 909 884 916 892 923 899 929 906 936 913 943 920 950 927 957 934 964 941 971 948 978 955 984 962 991 969 998 976 1005 983 1012 990 1019 997 1026 1004 1033 1011 1039 1018 1046 1025 1053 1032 1060 1039 1067 1046 1074 1053 1081 1060 1088 1067 1095 1074 1102 1081 1109 1087 1116 1094 1123 1101 1130 1109 1137 1115 1144 1122 1151 1129 1158 1136 1165 1142 1172 1149 1179 1156 1186 1162 1194 1169 1201 1176 1208 1182 1215 1189 1223 1195 1230 1202 1237 1209 1244 1216 1251 1222 1259 1229 1266 1235 1273 1242 1280 1248 1287 1255 1295 1261 1302 1268 1309 1274 1317 1281 1324 1287 1331 1294 1339 1300 1346 1307 1353 1313 1360 1320 1367 1326 1375 1333 1382 1340 1388 1346 1396 1353 1402 1358 1409 1365 1416 1372 1423 1378 1430 1384 1436 1391 1443 1397 1450 1403 1456 1410 1463 1416 1469 1422 1476 1428 1481 1434 1492 1444 1495 1447 1500 1453 1506 1458 1512 1464 1518 1470 1524 1476 1530 1482 1536 1488 1542 1494 1548 1500 1554 1506 1559 1511 1565 1517 1570 1522 1576 1527 1582 1533 1588 1539 1593 1544 1599 1549 1604 1555 1610 1560 1615 1565 1621 1571 1626 1576 1631 1581 1637 1586 1642 1591 1647 1596 1653 1601 1658 1606 1663 1611 1668 1615 1672 1620 1678 1625 1683 1630 1688 1635 1693 1640 1698 1645 1703 1649 1708 1654 1713 1658 1717 1663 1722 1667 1727 1672 1732 1676 1737 1681 1742 1686 1746 1690 1751 1694 1755 1699 1760 1703 1765 1708 1769 1712 1773 1717 1778 1721 1782 1725 1787 1730 1790 1733 1794 1738 1798 1742 1802 1745 1806 1749 1809 1753 1813 1756 1816 1760 1820 1763 1823 1766 1826 1770 1829 1773 1832 1776 1835 1779 1837 1781 1840 1784";
    	int skip = 2;
    	int count = 0;
    	int startingKickSkip = 50;
    	st = new StringTokenizer(path);
    	
    	try {	
    		for(int i = 0; i < startingKickSkip; i++) 
    		{
    			st.nextToken();
    			st.nextToken();
    		}
    		while(st.hasMoreTokens())
        	{
    			if(count++ == skip)
    			{
					double l = Double.parseDouble(st.nextToken());
					double r = Double.parseDouble(st.nextToken());
					left.add(l);
					right.add(r);
					count = 0;
    			}
    			else
    			{
    				try {
    				st.nextToken();
    				st.nextToken();
    				} catch(Exception e) {}
    			}
        	}
    	} catch(Exception e) {
    		e.printStackTrace();
    		Robot.drivetrain.stop();
    		DriverStation.reportWarning("NOOOO", true);
    	}
    	
    	/*
    	addParallel(new ZeroGyro());
    	addParallel(new ArmPincherClose());
    	addSequential(new Wait(.1));
    	addParallel(new ArmSetSwitch());
    	addSequential(new FollowEncoder(left, right));
    	addSequential(new DriveForward(.6, 1));
    	addSequential(new ArmPincherOpen());
    	addParallel(new SpintakeSet(.4,.4));
    	addSequential(new Wait(.2));
    	addParallel(new SpintakeStop());
    	addSequential(new DriveForward(-.3));
    	addParallel(new ArmSetLow());
    	addSequential(new TurnRight(62));
    	addParallel(new SpintakeSet(1,1));
    	addParallel(new SpintakePushIn());
    	addSequential(new DriveForward(.95, 1.3));
    	addSequential(new DriveForward(-.95-.2, 1.5));
    	addParallel(new SpintakeSet(-.5,.5));
    	addSequential(new Wait(.25));
    	addSequential(new SpintakePushOut());
    	addParallel(new SpintakeSet(1,1));
    	addParallel(new SpintakePushIn());
    	addSequential(new DriveForward(.2));
    	addSequential(new TurnLeft(62));
    	addParallel(new SpintakeStop());
    	addParallel(new SpintakePushOut());
    	addParallel(new ArmPincherClose());
    	addSequential(new Wait(.2));
    	addParallel(new ArmSetAngle(105));
    	addSequential(new DriveForward(.5, 1.3));
    	addSequential(new ArmPincherOpen());
    	*/
    	addSequential(new FollowEncoder(left, right));
    }
}
