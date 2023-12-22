# <img  style="float: left;" src="https://user-images.githubusercontent.com/23240128/233209820-821715e0-07e6-4dbc-8133-d915a7ea06b7.png" width=40> MyoSim: MyoSuite's musculoskeletal model library

 `MyoSim` is the library of MuJoCo Musculoskeletal Models of [MyoSuite](https://github.com/facebookresearch/myoSuite).

We stride to develop state of the art musculoskeletal model that can be physiologically accurate, experimentally validated, and stable for data driven esplorations. Models will be improved with feedback of the community and experiemntal validation. We provide a `grade` as a reference of the assesment performed so far:

| Grade | Expectation                                                 |
|-------|-------------------------------------------------------------|
| A+    | Grade `A` + Model reproduce functional experimental data  |
| A     | Grade `B` + Model matches literature data      |
| B     | Model is stable for data driven exploration   |
| C(*)  | Model can be loaded for testing         |

(*) Expect this model to be modified often until graduation to Grade `B`


## MyoSim Models

The models present in the library are:


| Grade   | Model name  |                    |
| :-----: |-------------|--------------------|
|    B    | **MyoFinger** <br> - 4 Degree of Freedom (DoF) <br> - 5 simplified muscles |<img src="https://user-images.githubusercontent.com/23240128/232323930-d1721f87-731b-432d-bafd-8c818ab4bbfe.png" width="200">|
|    A    | **MyoElbow**  <br> - 2 Degree of Freedom (DoF) <br> - 6 muscles | <img src="https://user-images.githubusercontent.com/23240128/232323890-6a601a82-1d3c-4e12-901c-0fd9cf232691.png" width="200">|
|    A    | **MyoHand**  <br>  - 23 Degree of Freedom (DoF) <br> - 39 muscles | <img src="https://user-images.githubusercontent.com/23240128/232323950-39552200-614b-4c73-aab5-8a78daa0f5f3.png" width="200">|
|    A    | **MyoLeg**  <br>  - 20 Degree of Freedom (DoF) <br> - 80 muscles | <img src="https://user-images.githubusercontent.com/12837145/236839645-e34eab3f-0358-4ca8-8ae0-68a5c08585e4.png" width="200">|
|    A    | **MyoArm**  <br>  - 27 Degree of Freedom (DoF) <br> - 63 muscles | <img src="https://github.com/MyoHub/myo_sim/assets/23240128/1f57c639-b7de-4bbb-a3c2-d2c29716e6c8" width="200">|
|    C    | **MyoLegX_Y**  <ul> <li> X Degree of Freedom (DoF)</li> <li> Y muscles</li> <li>with amputation + prostetic limb</li> </ul> | TBD |




Description of the models can be found [here](https://myosuite.readthedocs.io/en/latest/suite.html#models).


## License

MyoSuite is licensed under the [Apache License](LICENSE).

## Citation

If you find this repository useful in your research, please consider giving a star ⭐ and cite our [arXiv paper](https://arxiv.org/abs/2205.13600)  by using the following BibTeX entrys.

```BibTeX
@Misc{MyoSuite2022,
  author = {Vittorio, Caggiano AND Huawei, Wang AND Guillaume, Durandau AND Massimo, Sartori AND Vikash, Kumar},
  title = {MyoSuite -- A contact-rich simulation suite for musculoskeletal motor control},
  publisher = {arXiv},
  year = {2022},
  howpublished = {\url{https://github.com/facebookresearch/myosuite}},
  year = {2022}
  doi = {10.48550/ARXIV.2205.13600},
  url = {https://arxiv.org/abs/2205.13600},
}
```
