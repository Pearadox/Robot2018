package org.usfirst.frc.team5414.robot.commands;

import java.util.ArrayList;
import java.util.StringTokenizer;

import org.usfirst.frc.team5414.robot.Robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutonomousMidSwitch extends CommandGroup {

    public AutonomousMidSwitch() {
    	StringTokenizer st = null;
    	ArrayList<Double> left = new ArrayList<>();
    	ArrayList<Double> right = new ArrayList<>();
    	String path = "0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 1 1 1 1 2 1 2 2 3 2 3 3 4 3 5 4 5 5 6 6 7 6 8 7 9 8 10 9 12 11 13 12 14 13 15 15 17 16 18 18 20 19 21 20 23 22 24 24 26 25 28 27 30 29 31 30 33 32 35 34 37 36 39 38 41 40 44 42 46 44 48 46 51 49 53 51 56 54 58 56 61 59 64 61 66 64 69 67 72 70 76 73 79 76 82 79 85 82 89 85 92 88 95 91 99 94 102 97 106 101 110 104 113 107 117 111 121 114 125 118 129 121 135 126 138 128 142 131 146 135 151 139 156 142 162 146 167 150 172 154 178 158 184 162 190 166 196 169 202 173 209 178 215 182 222 186 229 190 236 194 242 199 249 203 256 207 264 212 271 216 278 220 286 225 293 229 301 234 308 238 316 242 323 246 331 251 338 255 346 260 354 265 361 269 369 274 376 279 383 284 390 289 397 293 405 298 412 303 419 308 425 313 432 319 439 323 445 328 452 333 458 338 464 343 470 348 476 353 481 357 487 362 493 367 499 372 505 377 510 381 516 386 521 390 526 395 532 399 537 404 542 408 547 413 552 417 556 421 561 426 566 430 571 435 575 439 579 443 584 447 588 451 592 455 596 459 600 463 604 467 609 471 612 474 616 477 619 481 623 485 627 489 630 492 634 496 637 499 640 502 643 506 646 509 650 513 653 516 656 519 659 522 662 525 665 529 668 532 670 535 673 538 676 541 679 544 681 547 684 550 686 552 689 555 691 558 693 561 695 563 698 566 700 568 702 571 704 574 707 576 709 579 710 581 712 584 714 586 716 589 718 591 719 593 721 595 723 598 724 600 726 602 727 604 729 606 730 608 731 609 732 611 734 613 735 615 736 617 737 618 738 620 739 621 740 623 741 625 741 626 742 628 743 629 743 630 744 632 744 634 745 635 745 637 745 639 745 640 746 642 746 644 746 645 746 647 746 649 746 650 746 652 746 654 746 655 746 657 746 659 746 660 746 662 746 664 746 665 746 667 746 668 746 670 746 671 746 673 746 674 746 676 746 677 746 678 746 680 746 681 746 682 746 683 746 684 746 685 746 686 746 687 746 688 746 688 746 689 746 690 746 691 746 691 746 692 746 692 746 693 746 693 746 694 746 694 746 694 746 694 747 695 747 695 747 695 747 695 747 695 747 695 747 695 747 695 747 695 747 695 747 695 747 695 747 695 747 695 747 695 747 695 747 695 747 695 747 695 747 695 747 695 747 695 747 695 747 695 747 695 747 695 747 695 747 695 747 695 747 695 747 695 747 695 747 696 747 696 747 696 747 696 747 696 747 697 747 697 748 697 748 698 748 698 748 699 749 700 749 700 749 701 749 702 750 703 750 704 751 704 751 705 752 706 752 707 753 708 753 709 754 710 755 711 755 712 756 713 757 713 758 714 758 715 759 716 760 717 761 718 762 719 763 720 764 721 765 722 766 723 766 724 767 725 768 726 769 727 770 728 771 730 772 731 773 732 774 733 775 734 776 735 777 737 778 738 779 739 780 740 782 741 783 742";
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
    	
    	addParallel(new ArmSetSwitch());
    	addSequential(new FollowEncoder(left, right));
    	addSequential(new SetAngle(0));
    }
}
