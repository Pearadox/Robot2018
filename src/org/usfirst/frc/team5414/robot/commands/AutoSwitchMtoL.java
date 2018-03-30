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
    	String path = "0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 1 1 1 1 2 1 2 1 2 1 3 2 3 2 4 2 4 3 5 3 5 4 6 4 6 4 7 5 8 6 9 6 9 7 10 7 11 8 12 9 13 9 14 10 15 11 15 12 16 13 17 14 18 15 19 16 20 17 20 18 21 19 22 21 23 22 23 23 24 25 25 26 25 28 26 29 27 31 28 33 28 35 29 36 30 38 30 40 31 42 32 44 32 46 33 49 34 51 34 53 35 55 36 57 36 59 37 61 38 63 39 66 40 68 41 70 42 72 43 74 44 75 45 77 46 79 47 81 49 83 50 85 51 86 53 88 54 90 56 92 57 93 58 95 60 97 61 99 62 101 64 103 65 105 66 107 67 109 69 111 70 113 71 115 73 117 74 119 75 121 76 123 77 125 79 127 80 130 81 131 82 134 83 136 84 138 85 140 86 142 87 144 88 146 90 149 91 151 92 153 93 155 94 157 95 160 96 162 97 164 98 166 99 169 100 171 101 173 102 175 103 178 105 182 105 184 106 185 107 187 107 190 108 192 109 195 110 197 111 200 111 202 112 205 113 207 114 210 115 212 115 215 116 217 117 220 117 222 118 225 119 228 119 230 120 233 121 236 121 238 122 241 122 244 123 247 123 250 123 253 124 256 124 259 124 262 124 265 124 268 124 271 124 274 124 278 124 281 124 284 124 287 124 290 124 293 124 295 124 298 124 301 124 304 124 307 124 310 124 313 124 316 124 319 124 321 124 323 125 326 125 328 125 331 126 333 126 336 126 338 127 340 127 342 128 344 129 346 129 348 130 350 130 352 131 354 132 356 133 358 133 360 134 361 135 363 136 365 137 366 138 368 139 370 140 371 141 373 142 374 143 375 144 376 146 377 147 378 149 380 151 381 152 381 154 382 156 383 158 384 159 385 161 386 163 387 164 387 166 388 168 389 169 390 171 391 172 392 174 392 175 393 177 394 178 395 180 396 181 397 182 398 183 399 185 400 185 402 186 403 187 404 188 405 189 407 190 408 190 410 191 411 192 413 192 414 193 416 193 417 193 419 194 421 194 423 194 424 195 426 195 428 195 430 195 431 195 433 195 435 196 436 197 438 197 440 197 441 198 443 199 444 199 445 200 447 201 448 201 449 202 451 203 452 204 453 205 454 206 456 207 457 208 458 209 459 210 460 210 461 211 462 212 463 213 464 214 465 215 466 216 467 217 468 218 469 219 470 220 471 222 472 223 473 224 474 225 475 227 476 228 478 229 479 230 480 232 481 233 483 235 484 236 485 238 486 239 488 241 489 242 490 244 492 246 493 247 494 249 495 251 497 252 498 254 499 256 501 258 502 259 503 261 505 263 506 265 508 267 509 269 510 270 512 272 513 274 515 276 516 278 518 280 519 282 521 284 522 286 524 288 525 290 527 292 528 294 530 296 531 298 533 300 534 302 536 304 537 307 539 309 540 311 542 314 543 316 545 318 546 321 548 323 549 326 551 329 553 331 554 334 556 337 557 340 559 343 561 346 562 350 564 353 565 356 567 360 568 363 570 367 572 370 573 374 575 378 576 381 578 385 579 389 581 392 582 396 584 400 585 403 587 407 588 411 590 415 592 419 593 422 595 426 597 430 598 434 600 437 602 441 604 445 605 448 607 452 609 456 611 459 613 463 615 466 617 470 619 473 621 477 623 480 625 484 627 488 629 491 631 495 633 499 635 502 637 506 639 510 641 513 643 517 645 521 646 524 648 528 650 532 651 535 653 539 655 543 656 547 658 551 660 554 661 558 663 562 665 566 667 570 668 573 670 577 672 581 673 584 675 588 677 591 679 595 681 598 682 602 684 605 686 609 688 612 690 616 692 619 694 623 696 626 697 629 699 633 701 636 703 639 705 643 707 648 710 650 712 654 714 657 716 660 718 663 720 666 722 669 724 673 726 676 729 679 731 682 733 686 736 689 738 692 740 696 743 699 745 702 747 705 750 709 752 712 754 715 756 718 759 722 761 725 763 728 766 731 768 735 770 738 773 741 775 744 777 747 780 750 782 754 784 757 787 760 789 763 792 766 794 769 797 773 799 776 801 779 804 782 806 785 809 788 811 791 814 795 816 798 819 801 821 804 824 807 826 810 829 814 831 817 834 820 836 823 839 826 841 829 843 832 846 835 849 839 851 842 854 845 856 848 859 851 861 854 864 857 866 860 869 863 872 866 874 869 877 872 879 875 882 878 885 881 887 884 890 888 893 891 895 894 898 897 901 900 903 902 906 906 909 909 912 912 914 915 917 917 920 920 923 923 925 925 928 926 929 927 930 928 930 929 930 931 930 932 930 933 930 935 931 936 931 937 931 939 932 940 932 941 933 942 933 942 934 941 934 941 934 940 934 940 934 940 934 940 934 941 935 941 935";
    	int skip = 3;
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
    	addSequential(new Wait(.2));
    	addParallel(new ArmSetSwitch());
    	addSequential(new FollowEncoder(left, right));
    	addParallel(new DriveForward(.4));
    	addSequential(new Wait(1));
    	addSequential(new ArmPincherOpen());
    	addSequential(new Wait(.5));
    	addSequential(new DriveForward(-1));
    	addSequential(new SetAngle(-90));
    	addSequential(new DriveForward(.7));
		*/
    	addSequential(new FollowEncoder(left, right));
    }
}
