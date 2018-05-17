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
    	String path = "0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 1 1 1 1 1 1 2 2 2 2 3 2 3 3 3 3 4 4 5 4 5 5 6 6 7 6 7 7 8 7 9 8 10 9 11 10 12 11 13 12 14 13 15 14 15 15 16 16 17 17 18 18 19 19 21 21 22 22 23 24 24 25 25 27 27 28 28 30 29 32 30 34 31 35 33 37 34 39 35 41 37 43 38 45 39 48 41 50 42 52 43 54 44 56 46 59 47 61 48 64 50 67 51 69 52 72 54 74 55 77 56 80 57 83 59 85 60 88 61 91 63 94 64 97 65 100 66 102 68 105 69 108 70 111 72 114 73 117 74 120 75 123 77 126 78 129 79 132 80 134 81 137 83 140 84 143 85 146 87 149 88 152 89 154 90 157 92 160 93 163 94 166 96 168 97 171 98 173 99 176 101 179 102 182 103 184 104 187 106 189 107 192 108 195 110 197 111 200 112 202 113 205 114 208 116 210 117 213 118 216 119 219 120 222 121 224 123 227 124 230 125 233 126 235 127 238 128 241 129 244 130 247 131 250 132 253 133 256 134 258 135 261 136 264 137 267 138 270 139 273 140 276 141 280 142 283 142 286 143 289 144 292 145 295 145 298 146 301 147 304 148 307 148 310 149 314 150 317 151 320 152 323 152 326 153 329 154 332 155 335 155 338 156 341 157 345 158 348 159 351 160 354 161 356 162 359 163 362 164 365 165 368 166 371 167 374 168 376 169 379 170 382 171 384 172 387 174 389 175 392 176 395 178 397 179 400 180 402 182 404 183 407 184 409 186 411 187 414 189 416 190 418 192 421 194 423 195 425 197 427 198 429 200 431 202 433 203 435 205 437 207 439 208 441 210 443 212 445 214 447 215 448 217 450 219 452 221 454 223 456 225 457 227 459 229 461 230 463 232 464 234 466 236 468 238 470 241 471 243 473 245 475 247 477 249 478 251 480 254 482 256 484 258 485 261 487 263 489 266 490 268 492 270 494 273 495 275 497 278 499 281 501 283 502 286 504 289 506 291 507 294 509 297 511 299 512 302 514 306 517 308 518 311 520 314 521 316 523 319 525 322 527 325 529 328 530 331 532 334 534 337 536 340 538 343 540 346 542 349 544 352 546 355 548 357 550 360 552 364 554 367 556 370 559 373 561 376 563 379 565 383 568 385 570 390 573 393 575 397 578 402 581 403 582 404 583 407 585 410 587 413 590 416 592 420 594 422 596 425 599 428 601 432 603 435 606 438 608 441 610 444 613 448 615 451 617 454 620 457 622 460 624 463 627 466 629 470 631 473 634 476 636 479 638 483 640 486 643 489 645 493 647 496 649 500 652 503 654 507 656 510 658 514 660 517 663 521 665 524 667 528 669 531 671 535 673 539 675 542 677 546 679 550 681 554 683 557 685 561 687 565 689 569 691 573 693 576 695 580 697 584 699 588 701 592 703 596 705 600 706 604 708 608 710 612 712 616 714 620 715 624 717 628 719 632 721 636 723 640 724 644 726 647 728 651 730 655 732 660 734 664 735 668 737 672 739 677 742 680 743 683 745 687 747 691 749 695 751 699 753 704 756 707 758 711 760 715 762 719 764 722 766 726 768 729 771 733 773 736 775 740 777 743 780 747 782 750 784 754 787 757 789 761 791 764 794 767 796 770 798 774 801 776 803 779 805 782 807 785 810 788 812 791 813 793 815 796 818 799 820 801 822 804 824 806 826 808 828 811 830 813 831 815 833 817 835 819 836 821 838 823 839 825 840 827 842 829 843 831 844 833 846 835 847 836 848 838 849 840 850 842 851 843 852 845 853 846 853 848 854 849 855 851 856 852 857 854 857 855 858 856 859 858 859 859 860 860 860 861 860 862 861 863 861 864 861 865 862 865 862 866 862 867 862 867 862";
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
    	
//    	/*
    	addParallel(new ZeroGyro());
    	addParallel(new ArmPincherClose());
    	addSequential(new Wait(.1));
    	addParallel(new ArmSetSwitch());
    	addSequential(new FollowEncoder(left, right));
    	addSequential(new DriveForward(1.3, 1.4));
    	addSequential(new ArmPincherOpen());
    	addParallel(new SpintakeSet(-.4,-.4));
    	addSequential(new Wait(.21));
    	addParallel(new SpintakeStop());
    	addSequential(new DriveForward(-.2));
    	addParallel(new ArmSetLow());
    	addSequential(new SetAngle(57));
    	addParallel(new SpintakeSet(1,1));
    	addParallel(new SpintakePushIn());
    	addSequential(new DriveForward(.95, 1.3));
    	addSequential(new DriveForward(-.8-.2, 1.5));
    	addParallel(new SpintakeSet(-.5,.5));
    	addSequential(new Wait(.45));
    	addSequential(new SpintakePushOut());
    	addParallel(new SpintakeSet(1,1));
    	addSequential(new DriveForward(.15));
    	addParallel(new SpintakePushIn());
    	addSequential(new SetAngle(0));
    	addParallel(new SpintakeStop());
    	addParallel(new SpintakePushOut());
    	addParallel(new ArmPincherClose());
    	addSequential(new DriveForward(-.3));
    	addParallel(new ArmSetAngle(105));
    	addSequential(new DriveForward(.9+.1, 1.3,4,3.3));
    	addSequential(new ArmPincherOpen());
//    	*/
//    	addSequential(new FollowEncoder(left, right));
    }
}
