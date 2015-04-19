
---


```













```

# Important Notice #

# The project is now moved to github, this page will be not be updated, and probably **soon suppressed**. #

Project page : http://jeremyfix.github.io/easykf/

Github source page : https://github.com/jeremyfix/easykf

```













```



---



**outdated from here**

## Introduction ##
EasyKF is a C++ library implementing the Extended Kalman Filter as well as the Unscented Kalman Filter. It is provided with several examples. There is also a document in which the filters are derived. This document is still in progress. Contributions and feedbacks are welcomed.

## Ressources ##

**Doxygen documentation** : [Link](http://easykf.googlecode.com/svn/doc/html/index.html)<br />
**Filters derivation and example python scripts** : see http://easykf.googlecode.com/svn/doc/Derivations

## Download ##

The new version 2.04 is available in sources only at [easykf-2.04.tar.gz](http://easykf.googlecode.com/svn/Sources/easykf-2.04.tar.gz)


## Other libraries for Kalman Filtering ##
- EKF/UKF Matlab toolbox http://www.lce.hut.fi/research/mm/ekfukf/

- R. Van Der Merwe ReBel Matlab Toolkit for KF/EKF/UKF : http://choosh.cse.ogi.edu/rebel/

- C++ KFilter for EKF : http://kalman.sourceforge.net/index.php


## Tutorials on Kalman filtering ##

- Detailed explanations on the filters : http://www.enotes.com/topic/Kalman_filter

- Lots of references on http://lewpayne.blogspot.com/2010/06/kalman-filters.html


## Example usage ##

Various examples are provided in the package :

- example-001 : learning the XOR with a 2-2-1 Multilayer perceptron with a logistic transfer function with UKF for parameter estimation with a scalar output,

- example-002 : learning the extended XOR with a 2-12-1 MLP with a logistic transfer function using UKF for parameter estimation with a scalar output. Check the paper "Adaptive Method of Realizing Natural Gradient Learning for Multilayer Perceptrons", Amari(1999) for a description of the problem,

- example-003 : learning a Radial Basis Function approximation of the sinc function with 10 kernels in 1D. We use UKF for parameter estimation with a scalar output to learn the amplitudes, mean and variance of the gaussian kernels,

- example-004 : learning the AND, OR , XOR boolean function with a 2-2-3 MLP with a sigmoid transfer function with UKF for parameter estimation with a vectorial output.

- example-005 : segfault so far (but the data files are probably not found); learning the MacKay robot arm data with a 2-12-2 MLP

- example-006 : minimizes the Rosenbrock banana function with UKF for parameter estimation

- example-007 : learning the parameters and state of a Lorentz attractor using joint UKF for state/parameter estimation,

- example-008 : [progress](In.md) should use square root UKF for state/parameter estimation of a Lorentz attractor

- example-009 EKF for state/parameter estimation in order to estimate the state and parameters of a Lorentz attractor
