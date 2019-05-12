## Build locally

```bash
docker build -t local-build --build-arg=<branch> .
```

**Note**: You can download the master branch with the following command.

```bash
docker build -t local-build .
```

### Run the container/start compilation

```bash
docker run -it local-build
colcon build --merge-install #Inside of the docker container
```
