FROM node:22-slim
WORKDIR /app
ENV NODE_ENV=production \
    PORT=8080
COPY package.json ./
RUN npm install --omit=dev --no-audit --no-fund
COPY server.mjs compare.py ./
COPY assets ./assets
COPY config ./config
COPY sim ./sim
COPY public ./public
EXPOSE 8080
CMD ["node", "server.mjs"]
