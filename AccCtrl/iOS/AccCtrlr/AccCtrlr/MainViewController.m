/*
     File: MainViewController.m
 based on 2013 Apple Inc. Code
 Abstract: Responsible for all UI interactions with the user and the accelerometer
 Version: 2.6
 */

#import "MainViewController.h"
#import "GraphView.h"
#import "AccelerometerFilter.h"

#define kUpdateFrequency	60.0
#define kLocalizedPause		NSLocalizedString(@"Pause","pause taking samples")
#define kLocalizedResume	NSLocalizedString(@"Resume","resume taking samples")

@interface MainViewController()
{
	AccelerometerFilter *filter;
	BOOL isPaused, useAdaptive;
}

@property (nonatomic, strong) IBOutlet GraphView *filtered;
@property (nonatomic, strong) IBOutlet UIBarButtonItem *pause;

- (IBAction)pauseOrResume:(id)sender;
- (IBAction)filterSelect:(id)sender;
- (IBAction)adaptiveSelect:(id)sender;

// Sets up a new filter. Since the filter's class matters and not a particular instance
// we just pass in the class and -changeFilter: will setup the proper filter.
- (void)changeFilter:(Class)filterClass;

@end

@implementation MainViewController

@synthesize filtered, pause;

// Implement viewDidLoad to do additional setup after loading the view.
- (void)viewDidLoad
{
	[super viewDidLoad];
    
	pause.possibleTitles = [NSSet setWithObjects:kLocalizedPause, kLocalizedResume, nil];
	isPaused = NO;
	useAdaptive = NO; //YES;
	[self changeFilter:[HighpassFilter class]];
	[[UIAccelerometer sharedAccelerometer] setUpdateInterval:1.0 / kUpdateFrequency];
	[[UIAccelerometer sharedAccelerometer] setDelegate:self];
	
//	[unfiltered setIsAccessibilityElement:YES];
//	[unfiltered setAccessibilityLabel:NSLocalizedString(@"unfilteredGraph", @"")];
//
	[filtered setIsAccessibilityElement:YES];
	[filtered setAccessibilityLabel:NSLocalizedString(@"filteredGraph", @"")];
}

// lock to landscape
- (NSUInteger) supportedInterfaceOrientations {
    return UIInterfaceOrientationMaskLandscapeLeft;
}

// UIAccelerometerDelegate method, called when the device accelerates.
- (void)accelerometer:(UIAccelerometer *)accelerometer didAccelerate:(UIAcceleration *)acceleration
{
	// Update the accelerometer graph view
	if (!isPaused)
	{
		[filter addAcceleration:acceleration];
		[filtered addX:filter.x y:filter.y z:filter.z];
	}
}

- (void)changeFilter:(Class)filterClass
{
	// Ensure that the new filter class is different from the current one...
	if (filterClass != [filter class])
	{
		// And if it is, release the old one and create a new one.
		filter = [[filterClass alloc] initWithSampleRate:kUpdateFrequency cutoffFrequency:5.0];
		// Set the adaptive flag
		filter.adaptive = useAdaptive;
	}
}

- (IBAction)pauseOrResume:(id)sender
{
	if (isPaused)
	{
		// If we're paused, then resume and set the title to "Pause"
		isPaused = NO;
		pause.title = kLocalizedPause;
	}
	else
	{
		// If we are not paused, then pause and set the title to "Resume"
		isPaused = YES;
		pause.title = kLocalizedResume;
	}
	
	// Inform accessibility clients that the pause/resume button has changed.
	UIAccessibilityPostNotification(UIAccessibilityLayoutChangedNotification, nil);
}

- (IBAction)filterSelect:(id)sender
{
	if ([sender selectedSegmentIndex] == 0)
	{
		// Index 0 of the segment selects the lowpass filter
		[self changeFilter:[LowpassFilter class]];
	}
	else
	{
		// Index 1 of the segment selects the highpass filter
		[self changeFilter:[HighpassFilter class]];
	}

	// Inform accessibility clients that the filter has changed.
	UIAccessibilityPostNotification(UIAccessibilityLayoutChangedNotification, nil);
}

@end
