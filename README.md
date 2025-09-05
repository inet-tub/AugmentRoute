# The Augmentation-Speed Tradeoff for Consistent Network Updates

This repository provides the code for [The Augmentation-Speed Tradeoff for Consistent Network Updates](https://arxiv.org/abs/2211.03716/) paper, published in SOSR2022.
For citation, please follow [this format](https://pourdamghani.net/_pages/SOSR22Tradeoff.bib).

## Paper abstract
Emerging software-defined networking technologies enable more adaptive communication infrastructures, allowing for quick reactions to changes in networking requirements by exploiting the workload’s temporal structure. However, operating networks adaptively is algorithmically challenging, as meeting networks’ stringent dependability requirements relies on maintaining basic consistency and performance properties, such as loop freedom and congestion minimization, even during the update process.
This paper leverages an augmentation-speed tradeoff to significantly speed up consistent network updates. 
We show that allowing for a small and short (hence practically tolerable, e.g., using buffering) oversubscription of links allows us to solve many network update instances much faster, as well as to reduce computational complexities (i.e., the running times of the algorithms). We first explore this tradeoff formally, revealing the computational complexity of scheduling updates. We then present and analyze algorithms that maintain logical and performance properties during the update. Using an extensive simulation study, we find that the tradeoff is even more favorable in practice than our analytical bounds suggest. In particular, we find that by allowing just 10% augmentation, update times reduce by more than 32% on average, across a spectrum of real-world networks.

## Requirments

### Required libraries

Please ensure to have a stable version of [NumPy](https://numpy.org/) and [pandas](https://pandas.pydata.org/) on your machine.
Furthermore, make sure to have the Python-complaint version of [Gurobi](https://www.gurobi.com/downloads/gurobi-software/) installed, and its license activated (we used its [academic liecense](https://www.gurobi.com/academia/academic-program-and-licenses/) in our experiments). 

### Required Dataset

In this paper, we used a 218 connected and directed graphs with a maximum of 100 nodes from [The Internet Topology Zoo](https://topology-zoo.org/dataset.html).

## Reproduction Steps

- Please first load the zoo topologies in the [Input Generation](./InputGen) folder.
- Then, in the "inputGen.py" file, adjust the parameters (described in the paper) based on your needs, to get an instance of the probkem.
- Afterwards, by running "running.py" on the generated instance, you would get results in the excel format.
- Lastly, feed the excel format to the desired visluzation code to get a final plot. 
