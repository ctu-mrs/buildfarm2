ARG BUILDER_IMAGE=set_from_outside
ARG BASE_IMAGE=set_from_outside

#############################################################################

FROM $BUILDER_IMAGE AS update_builder

COPY /tmp/debs /tmp/tmp_debs

RUN ls /tmp/tmp_debs && \
    apt-get update && apt-get -y install /tmp/tmp_debs/*.deb && \
    rm -rf /tmp/tmp_debs

#############################################################################

FROM $BASE_IMAGE AS squash_builder

COPY --from=update_builder / /
