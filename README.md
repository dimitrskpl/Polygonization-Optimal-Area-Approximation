# Polygonization Optimal Area Approximation

**Problem:** Polygonization of a point-set **S** with **n** points in $\mathbf{R}^2$, and optimal (minimal or maximal) area approximation.  
This is an extension of [this work](https://github.com/rondojim/Point-Set-Polygonization).

The implemented algorithms are based on the following papers:  
- [Local Search algorithm](https://en.wikipedia.org/wiki/Local_search_(optimization)) from [1]
- [Simulated Annealing algorithm](https://en.wikipedia.org/wiki/Simulated_annealing) from [2]
- [Ant Colony Optimization](https://en.wikipedia.org/wiki/Ant_colony_optimization_algorithms) from [3]

![](optimal-polygonizations.png)

A detailed report of this project it is provided in the `Report.pdf` file.

## Usage
**Dependency:** Make sure you have CGAL installed, or download it from [here](https://www.cgal.org/download.html).  

Go to `build/` directory and run:

```
$ cmake .. && make
$ ./optimal_polygon -i <input file> -o <output file> 
  -algorithm <local_search/simulated_annealing/ant_colony> 
  -init_algo <onion/ch2poly | except ant_colony>
  -L [L parameter according to algorithm]  
  –max [maximal area polygonization]  
  –min [minimal area polygonization]  
  –threshold <double only in in local search>  
  –annealing <local/global/subdivision only in simulated annealing>
  -alpha <double> -beta <double> -rho <double> elitism <0 or 1> 
  [alpha, beta, rho, elitism only in ant_colony]
```

## References

[1]: [Greedy and Local Search Heuristics to Build Area-Optimal Polygons](https://arxiv.org/abs/2106.14728)   
[2]: Nir Goren, Efi Fogel, and Dan Halperin. 2022. Area Optimal Polygonization Using Simulated Annealing. ACM J. Exp. Algorithmics 27, Article 2.3 (December 2022), 17 pages. https://doi.org/10.1145/3500911  
[3]: Taranilla, M.T., Gagliardi, E.O., & Peñalver, G.H. (2011). [Approaching minimum area polygonization.](https://oa.upm.es/19287/1/INVE_MEM_2011_121744.pdf)


## Authors
* [Kyriakopoulos Dimitrios](https://github.com/dimitrskpl)<sup>*</sup>
* [Poulis Angelos](https://github.com/angelosps)<sup>*</sup>  
* [Rontogiannis Dimitrios](https://github.com/rondojim)<sup>*</sup>


<sup>*</sup>_Equal contribution._

