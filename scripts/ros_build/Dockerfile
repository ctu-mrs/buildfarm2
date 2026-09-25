# syntax=docker/dockerfile:1
ARG BUILDER_IMAGE=set_from_outside
ARG BASE_IMAGE=set_from_outside

#############################################################################

FROM $BUILDER_IMAGE AS update_builder

# Secret contents do not affect the BuildKit cache key. Separate public and private installs.
ARG PRIVATE_PPA_CACHE_SCOPE=public

COPY /tmp/debs /tmp/tmp_debs

RUN --mount=type=bind,source=add_private_ppa.sh,target=/tmp/add_private_ppa.sh \
    --mount=type=bind,source=install_debs.sh,target=/tmp/install_debs.sh \
    --mount=type=secret,id=PRIVATE_PPA_TOKEN \
    : "$PRIVATE_PPA_CACHE_SCOPE" && bash /tmp/install_debs.sh

#############################################################################

FROM $BASE_IMAGE AS squash_builder

COPY --from=update_builder / /
