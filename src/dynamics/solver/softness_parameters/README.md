# Soft Constraints

Physics engines use **constraints** to enforce rules describing physical interactions between bodies.
For example, a contact constraint can be used to push colliding bodies apart and solve overlap,
and a distance joint might enforce a limit on the distance between objects.

Physics simulations often model constraints as **springs** that can be tuned with stiffness
and damping coefficients. However, these kinds of traditional springs can be prone to instability,
and the parameters can be highly unintuitive, requiring knowledge of the mass to get a specific outcome.

**Soft constraints** aim to solve this issue, being stable and easy to tune. They can be seen
as an evolution of **Baumgarte stabilization**, a way to boost contact impulses to account for overlap.

Soft constraints are based on the harmonic oscillator, dampening the constraint response
with intuitive tuning parameters expressed only by the frequency in Hertz, a non-dimensional
damping ratio, and the time step. This formulation was popularized by Erin Catto,
the author of [Box2D], and simplified further by Ross Nordby, the author of [Bepu].

```rust,ignore
// Parameters
let zeta = 1.0;               // Damping ratio (controls the amount of oscillation)
let hertz = 5.0;              // Frequency (cycles per second)
let omega = 2.0 * PI * hertz; // Angular frequency (controls the rate of oscillation)

// Shared expressions
let a1 = 2.0 * zeta + omega * delta_time;
let a2 = omega * delta_time * a1;
let a3 = 1.0 / (1.0 + a2);

// Coefficients
let bias_coefficient = omega / a1;
let mass_coefficient = a2 * a3;
let impulse_coefficient = a3;
```

The bias coefficient has the unit of the inverse of time,
while the impulse and mass coefficients are non-dimensional.

## Contact Impulses

Without soft constraints or Baumgarte stabilization, the contact impulse for the current iteration
can be computed with the following formula using *Projected Gauss-Seidel* (PGS):

```rust,ignore
let incremental_impulse = -effective_mass * normal_speed;
```

where `effective_mass` is the mass "seen" by the constraint along the contact normal,
and `normal_speed` is the relative velocity along the contact normal.

With Baumgarte stabilization, an additional bias term is added to account for overlap:

```rust,ignore
let bias = bias_factor / delta_time * penetration_depth.max(0.0);
let incremental_impulse = -effective_mass * (normal_speed + bias);
```

where `bias_factor` determines how much of the overlap is solved
within a single step, typically in the [0.1, 0.3] range.

Soft constraints modify the expression further by incorporating the softness
parameters, dampening the response in a stable and controlled manner:

```rust,ignore
let biased_normal_speed = normal_speed + bias_coefficient * separation;
let scaled_effective_mass = mass_coefficient * effective_mass;
let extra_impulse = impulse_coefficient * accumulated_impulse;

let incremental_impulse = -scaled_effective_mass * biased_normal_speed - extra_impulse;
```

1. `biased_normal_speed` is the relative velocity along the contact normal,
boosted by the separation distance scaled by the `bias_coefficient`.

2. `scaled_effective_mass` is the mass "seen" by the constraint along the contact normal,
scaled by the `mass_coefficient`.

3. Finally, the accumulated impulse is multiplied by the `impulse_coefficient`
and subtracted from the incremental impulse to prevent the total impulse
from becoming too large.

The result is a contact constraint that is highly stable and easy to tune
with intuitive frequency and damping ratio parameters.

## Relation to CFM and ERP

Soft constraints are not a new concept, dating back to at least
the [Open Dynamics Engine (ODE)](https://www.ode.org/).

In ODE, constraints use tuning parameters called **CFM** and **ERP**.
These are also present in some other physics engines such as [Rapier](https://rapier.rs).

CFM is a *Constraint Force Mixing* parameter. It essentially controls
how soft or hard a constraint is. A CFM of `0.0` represents a hard constraint,
and the softness of the constraint increases as the CFM increases.

ERP is an *Error Reduction Parameter*. It determines how much of the constraint error
is fixed during a simulation step. For example, for a joint, an ERP of `0.0` means
that the joint applies no correction, while an ERP of `1.0` means that the joint
will attempt to fix all error in a single step.

The CFM and ERP actually have a connection to the spring stiffness and damping constant:

```rust,ignore
let cfm = 1.0 / (delta_time * stiffness + damping);
let erp = (delta_time * stiffness) / (delta_time * stiffness + damping);
```

The `stiffness` and `damping` on the other hand can be computed based on
the frequency and damping ratio introduced earlier:

```rust,ignore
let stiffness = effective_mass * omega * omega;
let damping = 2.0 * effective_mass * zeta * omega;
```

This means that the CFM and ERP can also be computed based on the frequency and damping ratio.
However, using the previously shown formulae for `bias_coefficient`, `impulse_coefficient`,
and `mass_coefficient`, we can represent soft constraints with properties that are independent
of mass properties, and get around having to explicitly compute CFM and ERP.

## References

- Erin Catto. Feb 5, 2024. [Solver2D - Soft Constraints](https://box2d.org/posts/2024/02/solver2d#soft-constraints)
- Erin Catto. GDC 2011. [Soft Constraints - Reinventing the Spring](https://box2d.org/files/ErinCatto_SoftConstraints_GDC2011.pdf)
- Erin Catto. GDC 2006. [Sequential Impulses]
- [Open Dynamics Engine (ODE) Manual](http://ode.org/wiki/index.php/Manual)

[Box2D]: <https://box2d.org/>
[Bepu]: <https://www.bepuentertainment.com/>
[Sequential Impulses]: <https://box2d.org/files/ErinCatto_SequentialImpulses_GDC2006.pdf>
