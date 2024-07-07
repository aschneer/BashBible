# poetry

## References

Basic usage: [https://python-poetry.org/docs/basic-usage/](https://python-poetry.org/docs/basic-usage/)

## Create New Project

If git repo already exists:

```bash
cd repo_name
poetry init
```

If git repo doesn't exist yet:

```bash
poetry new repo_name
cd repo_name
git init
```

## Add PyPi/pip Dependencies

```bash
poetry add <package>
# example
poetry add jupyter
```

## Add Packages from Private Repository

[https://python-poetry.org/docs/repositories/](https://python-poetry.org/docs/repositories/)

Add private repository:

```bash
# repository_name is a name that will be
# used to refer to the private repository.
poetry source add --priority=supplemental repository_name https://my.repo.com/repo
```

Add package from private repository:

```bash
poetry add --source repository_name private_package_name
```

## Run Jupyter Notebook from Poetry

```bash
poetry add jupyter # if not already done
poetry shell
jupyter notebook
# when done with shell...
exit
```
