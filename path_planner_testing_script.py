"""
This script should assist in validating the performance of the path planning systems I've developed. 
I want to consider the path planner's performance at 15 distinct input start-goal pairs. 5 of these start goal pairs will be invalid. 10 will be valid.
For each pairing, I want to do this 10 times:
  I want to plot the state space and also the return path. Then I want to consider, qualitatively, what I think and record a summary of those observations at the end of the 10 trials.
  I want to compute these performance metrics for the trial since these are what I initially considered as being important: time to compute global path, optimality compared to 1M resolution 3D grid with A* and euclidean h, the sparsity ratio nf/n, and the safety as measured by the mean distance from obstacle boundaries
 I want to charachterize the central tendency of my best estimate of the generalized results: a box plot with the median and middle 50% for each path planning system given all 10 valid routes    

"""