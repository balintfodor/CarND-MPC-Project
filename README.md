# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

[The original project README.md](https://github.com/udacity/CarND-MPC-Project/blob/master/README.md)

---

## Results

Screen capture of the final version:
[https://www.youtube.com/watch?v=9isuVcoaZls&feature=youtu.be](https://www.youtube.com/watch?v=9isuVcoaZls&feature=youtu.be)

## Project Rubrics

### Compilation

#### cmake

The only changes I made to the `CMakeLists.txt` is adding `poly.cpp` as an additional source to be compiled. `poly.h` and `poly.cpp` contains the polynomial fit and eval algorithms that were originally contained in `main.cpp`.

#### Sidenotes on Ipopt (MacOs)

* `homebrew/science` is not availabe anymore
* `ipopt` is not yet migrated to `homebrew/core`
* `mumps.rb` (which is a dependency of `ipopt.rb`) maintained at the original project repo depends on `scotch5`
* `scotch5` is not yet migrated to `homebrew/core`

I could not manage to get `Ipopt` using `homebrew`, instead I compiled it from source manually. `install_ipopt.sh` failed silently for me. `./configure` gave error when tested `blas` with the option `--with-blas="$prefix/lib/libcoinblas.a -lgfortran"`.

After downloading and extracting `Ipopt-3.12.10`, in the `Ipopt-3.12.10` directory run:

    ./ThirdParty/ASL/get.ASL
    ./ThirdParty/Blas/get.Blas
    ./ThirdParty/Lapack/get.Lapack
    ./ThirdParty/Mumps/get.Mumps
    ../configure --prefix=/usr/local
    sudo make install

### Implementation

The model is the same as presented in the lectures: 6 state parameters (`x, y, psi, v, cte, epsi`) and 2 actuators (steering angle and throttle). The update equations are the same as in the lectures.

`T=2s` seems reasonable for the prediction time range. Based on that I empirically chose `dt=0.05s` and `N=20`. `dt` seems fine enough but `N` is not so large to give the solver hard times.

I measured the computational latency (including the arbitrary `sleep_for` call). On my computer, it was around 120 milisecs when no other high demanding app is running. Hence the `const double expected_latency = 0.12;` variable in the code.

After receiving the waypoints, using the kinematics equations (`globalKinematicStep, MPC.h`) and the expected latency, the state is updated. As there can not be a fully exact prediction for the state, the waypoints drawn back to the simulator can 'shake'. Especially when the steering angle is high. In general, we can expect that the distant waypoints are the most inaccurate ones.

Based on this new state, the waypoints are transformed into the vehicle coordinate system. A 3rd order polynomial is fit to the waypoints, so the initial `cte` and `epsi` can be calculated.

One of the most important sections is the cost function definition. We construct the cost function as the sum of several penalties. The penalties can be grouped by timesteps. We gradually decrease the weights of all the penalties of a group as we move on the prediction timeline since we expect that we can predict more accurately when we calculating closer things in space and time.

For every timestep the following penalties are taken into consideration (all of them are factored by its timestep weight):

* penalty for high `cte`
* penalty for high `epsi`
* reward for high velocity
* penalty for high steering angle, this penalty is increasing with the velocity, so we expect much smaller turns at higher speeds
* high throttle penalty, this penalty is increasing with the `cte`, so we expect to slow down when `cte` is high
* strong penalty on `cte` change, we want to avoid oscillating around the waypoint polynomial, this term try to force the solver to favor solutions that bring `cte` down smoothly
* strong penalty on `epsi` change, same as for the previous one
* penalty on huge turns, we want to drive smoothly
* penalty on hectic throttling, we want to drive smoothly

The results can be seen in
[https://www.youtube.com/watch?v=9isuVcoaZls&feature=youtu.be](https://www.youtube.com/watch?v=9isuVcoaZls&feature=youtu.be)
