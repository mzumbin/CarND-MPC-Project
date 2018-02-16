# Report

###  State
the state is composed by [ px,py,psi,v,cte,epsi], this project uses the veichele coordinate reference and because that, the first state always start with px,py and psi equals to zero
 
```c++
Eigen::VectorXd state(6);
state << 0, 0, 0, v, cte, epsi;
```

### Actuator and Model
The actuators are delta and aceleration using the cvtrl model with equations below

the update equation and actuators are used in the code at  FG_eval () operator , the model and actuators are in the code below, the update state is used as a constrain in ipopt solver.

```c++
// x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
// y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
// psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
// v_[t+1] = v[t] + a[t] * dt
// cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
// epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
            
fg[2 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
fg[2 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
fg[2 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
fg[2 + v_start + t] = v1 - (v0 + a0 * dt);
fg[2 + cte_start + t] =
 cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
fg[2 + epsi_start + t] =
                    epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
                    fg[2 + epsi_start + t] =
                    epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
```

### Optimization Objective Function
The cost function try to minimize the primarily the cte and  epsi, this are achieved by using greater weights contants than other variables as  reference velocity , the actuators and the gap between actuators, 
the minimizatizon errors can be seen in the code below

```c++
 // The part of the cost based on the reference state.
        for (int t = 0; t < N; t++) {
            fg[0] += 2000*CppAD::pow(vars[cte_start + t], 2);
            fg[0] += 2000*CppAD::pow(vars[epsi_start + t], 2);
            fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
        }

        // Minimize the use of actuators.
        for (int t = 0; t < N - 1; t++) {
            fg[0] += 5*CppAD::pow(vars[delta_start + t], 2);
            fg[0] += 5*CppAD::pow(vars[a_start + t], 2);
        }

        // Minimize the value gap between sequential actuations.
        for (int t = 0; t < N - 2; t++) {
            fg[0] += 200*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
            fg[0] += 10*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
        }
```        



### Timestep Length and Elapsed Duration (N & dt)
the values used are sugested in the Q&A video and they worked great

### Polynomial Fitting and MPC Preprocessing
As shown in the state, the coordinates used are the cars coordinate. So the waypoints must be transformed, the code above shows the trasformation. the code also shows the aproximation used in cte( distance to y , not the smalest distance to waypoint polynomial) and the epsi wich is just the slope of the polynomial evaluated at 0;

```c++

for (int i = 0; i < ptsx.size(); i++) {
  double x = ptsx[i] - px;
  double y = ptsy[i] - py;

   ptsx[i] = x * cos(-psi) - y * sin(-psi);
   ptsy[i] = x * sin(-psi) + y * cos(-psi);
 }
   Eigen::Map<Eigen::VectorXd> pts_x_trans(&ptsx[0], 6);
  Eigen::Map<Eigen::VectorXd> pts_y_trans(&ptsy[0], 6);
  auto coefs = polyfit(pts_x_trans, pts_y_trans, 3);
// atan(derivative of poly eval at zero)
double epsi = -atan(coefs[1]); 
                
```


### Model Predictive Control with Latency
The predicition uses the same model to predict the 100ms ahead , the velocity is calculated using the trapeziodal rule(https://en.wikipedia.org/wiki/Trapezoidal_rule), so v = (v(t-1)+v(t)) /2 and the update eq becomes:

```c++
//  v  * 0.44704 mph to m/s
v = (v+last_v)/2  ;
psi = psi + v  * 0.44704 * delta / Lf * 0.1 ;
px = px + v  * 0.44704 * cos(psi) * 0.1 ;
py = py + v * 0.44704  * sin(psi) * 0.1 ;
last_v =v;

```
