# Absolute-Stability-of-Weakly-Nonlinear-Systems


## Motivation
Control of a distributed networks of agents is one of most important topics in contemporary systems and control theory, and has application to a wide variety of practical systems such as flocking in autonomous vehicle swarms, and coordination of satellites. The theory is also relevant to all control systems which interact over some communication network such as the internet or Bluetooth, or similar. Unfortunately, classical controller design techniques are \emph{inherently unsuitable} for achieving consensus in networks since the networks are often multi-variable in nature and contain various non-standard constraints associated with the network structure. Therefore, over the past decade or so, theories have been developed to cope with the new challenges presented by such systems. 


## Problem
A distributed system is a group of network components that work together to act as a single entity. For them to act as a single entity they must agree on `some' data value i.e. 
consensus.  To obtain this, there are algorithms which instruct each agent, also known as consensus protocols. These protocols must be fault tolerant, resilient, and designed to deal with a limited number of faulty processes.  ~\\
The majority of these algorithms make the strong assumption that the plant is \emph{linear}. Whereas in reality, all practical control systems are \emph{nonlinear} to some degree, with many being representable, approximately by the state-space equations:
\begin{align*}
 \dot x(t) & = Ax(t) + B\psi \big( u(t) \big) \\
       y(t) & = C x(t)
\end{align*}
where the function $\psi(\cdot)$ is a static non-linearity, belonging to some loosely defined class, with not all information about this function known. The above class of systems is similar to a standard linear state-space system but exhibits much richer behaviour due to the presence of $\psi(\cdot)$. ~ \\
The work will explore the properties of $\psi(\cdot)$, and the amount of knowledge required of this function, in order for networks of this type of system to reach a consensus. An observer for the weakly non-linear system above was proposed in [1] and will be used as the foundation of an approach towards consensus algorithm construction. A key difficulty which will require careful consideration is that of determining the required amount of information (we know about a system) in order for the system to reach consensus. ~\\
A further related area of interest is that of the consensus algorithms \emph{optimality}: in what sense is the algorithm optimal; what is the nature of the trade-off between optimality and information sharing in networks?  Designing optimal consensus control laws and adopting different techniques (such as integral quadratic constraints) will exploit ideas found in [1-3].

# Goals
The work has three major technical goals, 
\begin{enumerate}
\itemsep-0.05em
 \item To develop consensus control laws for networks of the system given above. Special attention will be devoted to uncovering the minimal amount of knowledge of $\psi(\cdot)$ required; this is an open theoretical problem. 
 \item To explore optimality properties of the consensus algorithms and obtain conditions which ensure that cost functions of the form
 \[
  \int_0^{\infty} L(x,u,t) ~ dt
 \]
 are minimised. Special attention will be devoted to uncovering the relationship between optimality, the underlying graph structure of the network and and the properties of $\psi(\cdot)$. 
 \item The simulation and demonstration of the network systems and consensus algorithms in MATLAB and Simulink
 
\end{enumerate}~\\
The goals are not binary and should yield interesting results, possibly suitable for conference publication, which is a stretch goal of the scholarship.
