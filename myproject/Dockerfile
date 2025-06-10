# これらのうち 1 行だけコメントアウトしない
# FROM python:3.12.3-bullseye
# FROM python:3.11.9-bullseye
FROM python:3.10.14-bullseye
# FROM python:3.9.19-bullseye
# FROM python:3.8.19-bullseye

# 古いpoetryが入っている可能性があり、build時にpoetry exportが出来ずエラーになる
# RUN pip install poetry

# Poetry 1.7.1 を明示的にインストール
ENV POETRY_VERSION=1.7.1
RUN curl -sSL https://install.python-poetry.org | python3 - && \
    ln -s /root/.local/bin/poetry /usr/local/bin/poetry

RUN poetry config virtualenvs.create false

WORKDIR /code
COPY ./pyproject.toml /code/
RUN poetry export --without-hashes --dev --output requirements.txt
RUN pip install -r requirements.txt

COPY . /code/
ENV PYTHONPATH=/code

CMD ["python", "main.py"]
