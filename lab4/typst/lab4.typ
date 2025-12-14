#import "appendices.typ": appendices

// charged-ieee
#import "@preview/charged-ieee:0.1.4": ieee

#show: ieee.with(
  title: [Lab 4 Report: EXERCISES],
  authors: (
    (
      name: "Yilin Zhang, 23020036094, Group 31",
      department: [School of Computer Science],
      organization: [Ocean University of China],
      location: [Qingdao, China],
      email: "zyl8820@stu.ouc.edu.cn"
    ),
  ),
  // bibliography: bibliography("refs.yml"),
  figure-supplement: [Fig.],
)

// equate
#import "@preview/equate:0.3.2": equate

// #show: equate

// Display block code in a larger block
// with more padding.
#show raw.where(block: true): block.with(
  width: 100%,
  fill: luma(240),
  inset: 10pt,
  radius: 4pt,
)

#show link: underline
#set math.mat(delim: "[")
#set math.vec(delim: "[")
#set math.equation(supplement: none, numbering: "(1)")

= Individual

#bibliography("refs.yml")

// Display inline code in a small box
// that retains the correct baseline.
#show raw.where(block: false): box.with(
  fill: luma(240),
  inset: (x: 3pt, y: 0pt),
  outset: (y: 3pt),
  radius: 2pt,
)

// Reset raw blocks to the same size as normal text,
// but keep inline raw at the reduced size.
#show raw.where(block: true): set text(0.8em)

#show: appendices

= Individual

== Deliverable - Single-segment trajectory optimization

Consider the following minimum velocity ($r = 1$) single-segment trajectory optimization problem:

#equate[
  $ min_P(t) integral_0^1(P^((1))(t))^2 d t $ &, <obj> \
  $                      s.t. quad P(0) = 0 $ &, <bc0> \
  $                                P(1) = 1 $ &, <bc1>
]

with $P(t) in RR[t]$, i.e., $P(t)$ is a polynomial function in $t$ with real coefficients:

$
  P(t) = p_N t^N + p_(N-1) t^(N-1) + dots + p_1 t + p_0.
$

Note that because of constraint (@bc0), we have $P(0) = p_0 = 0$, and we can parametrize $P(t)$ without a scalar part $P_0$.

_1._ Suppose we restrict $P(t) = p_1 t$ to be a polynomial of degree 1, what is the optimal solution of problem (@obj)? What is the value of the cost function at the optimal solution?

_2._ Suppose now we allow $P(t)$ to have degree 2, i.e., $P(t) = p_2 t^2 + p_1 t$.

_(a)_ Write $min_P(t) integral_0^1(P^((1))(t))^2 d t$, the cost function of problem (@obj), as $bold(p)^top bold(Q) bold(p)$, where $bold(p) = [p_1, p_2]^top$ and $bold(Q) in bold(S)^2$ is a symmetric $2 times 2$ matrix.

_(b)_ Write $P(1) = 1$, constraint (@bc1), as $bold(A) bold(p) = bold(b)$, where $bold(A) in RR^(1 times 2)$ and $bold(b) in RR$.

_(c)_ Solve the Quadratic Program (QP):

$
  min_bold(p) bold(p)^top bold(Q) bold(p) quad s.t. quad bold(A) bold(p) = bold(b).
$ <qp>

You can solve it by hand, or you can solve it using numerical QP solvers (e.g., you can easily use the #link("https://www.mathworks.com/help/optim/ug/quadprog.html")[`quadprog`] function in Matlab). What is the optimal solution you get for $P(t)$, and what is the value of the cost function at the optimal solution? Are you able to get a lower cost by allowing $P(t)$ to have degree 2?

_3._ Now suppose we allow $P(t) = p_3 t^3 + p_2 t^2 + p_1 t$:

_(a)_ Let $bold(p) = [p_1, p_2, p_3]^top$, write down $bold(Q) in bold(S)^3$, $bold(A) in RR^(1 times 3)$, $b in RR$ for the QP @qp.

_(b)_ Solve the QP, what optimal solution do you get? Do this example agree with the result we learned from Euler-Lagrange equation in class?

_4._ Now suppose we are interested in adding one more constraint to problem (@obj):

$
  min_p(t) integral_0^1(P^((1))(t))^2 d t, s.t.
  P(0) = 0, P(1) = 1, P^((1))(1) = -2.
$ <ooobj>

Using the QP method above, find the optimal solution and optimal cost of problem (@ooobj) in the case of:

_(a)_ $P(t) = p_2 t^2 + p_1 t$, and

_(b)_ $P(t) = p_3 t^3 + p_2 t^2 + p_1 t$.
