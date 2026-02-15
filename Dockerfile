# Base image with Python and essential build tools
FROM mambaorg/micromamba:latest

USER root

# Install system dependencies for CadQuery and OpenCASCADE
RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx \
    libglu1-mesa \
    libx11-6 \
    libxrender1 \
    libxext6 \
    libice6 \
    libsm6 \
    libfontconfig1 \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /app

# Install dependencies using micromamba (best for cadquery/ocp)
RUN micromamba install -y -n base -c conda-forge \
    python=3.10 \
    cadquery \
    scipy \
    numpy \
    flask \
    flask-cors \
    gunicorn \
    aerosandbox \
    && micromamba clean --all --yes

# Copy project files
COPY . .

# Environment variables
ENV PATH="/opt/conda/bin:$PATH"
ENV FLASK_APP=api.py

# Expose the port Railway will provide
EXPOSE 5000

# Start the API using Gunicorn for production stability
CMD ["micromamba", "run", "-n", "base", "gunicorn", "--bind", "0.0.0.0:5000", "api:app"]
