const fz = require('zigbee-herdsman-converters/converters/fromZigbee');
const tz = require('zigbee-herdsman-converters/converters/toZigbee');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const reporting = require('zigbee-herdsman-converters/lib/reporting');
const e = exposes.presets;
const ea = exposes.access;

const tzLocal = {
    node_config: {
        key: ['report_delay', 'comparison_previous_data'],
        convertSet: async (entity, key, rawValue, meta) => {
            const lookup = {'OFF': 0x00, 'ON': 0x01};
            const value = lookup.hasOwnProperty(rawValue) ? lookup[rawValue] : parseInt(rawValue, 10);
            const payloads = {
                //report_delay: ['genPowerCfg', {0x0201: {value, type: 0x21}}],
		comparison_previous_data: ['genPowerCfg', {0x0205: {value, type: 0x10}}],
            };
            await entity.write(payloads[key][0], payloads[key][1]);
            return {
                state: {[key]: rawValue},
            };
        },
    },
	termostat_config: {
        key: ['high_temp', 'low_temp', 'enable_temp', 'invert_logic_temp','report_delay'],
        convertSet: async (entity, key, rawValue, meta) => {
            const lookup = {'OFF': 0x00, 'ON': 0x01};
            const value = lookup.hasOwnProperty(rawValue) ? lookup[rawValue] : parseInt(rawValue, 10);
            const payloads = {
                high_temp: ['msTemperatureMeasurement', {0x0221: {value, type: 0x29}}],
                low_temp: ['msTemperatureMeasurement', {0x0222: {value, type: 0x29}}],
                enable_temp: ['msTemperatureMeasurement', {0x0220: {value, type: 0x10}}],
                invert_logic_temp: ['msTemperatureMeasurement', {0x0225: {value, type: 0x10}}],
                report_delay: ['msTemperatureMeasurement', {0x0201: {value, type: 0x21}}],
            };
            await entity.write(payloads[key][0], payloads[key][1]);
            return {
                state: {[key]: rawValue},
            };
        },
    },
	temperaturef_config: {
        key: ['temperature_offset'],
        convertSet: async (entity, key, rawValue, meta) => {
            const value = parseFloat(rawValue)*10;
            const payloads = {
                temperature_offset: ['msTemperatureMeasurement', {0x0210: {value, type: 0x29}}],
            };
            await entity.write(payloads[key][0], payloads[key][1]);
            return {
                state: {[key]: rawValue},
            };
        },
    },
};

const fzLocal = {
    node_config: {
        cluster: 'genPowerCfg',
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            const result = {};
            if (msg.data.hasOwnProperty(0x0201)) {
                result.report_delay = msg.data[0x0201];
            }
	    if (msg.data.hasOwnProperty(0x0205)) {
		result.comparison_previous_data = ['OFF', 'ON'][msg.data[0x0205]];
            }
            return result;
        },
    },
	termostat_config: {
        cluster: 'msTemperatureMeasurement',
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            const result = {};
            if (msg.data.hasOwnProperty(0x0221)) {
                result.high_temp = msg.data[0x0221];
            }
	    if (msg.data.hasOwnProperty(0x0222)) {
                result.low_temp = msg.data[0x0222];
            }
            if (msg.data.hasOwnProperty(0x0220)) {
                result.enable_temp = ['OFF', 'ON'][msg.data[0x0220]];
            }
            if (msg.data.hasOwnProperty(0x0225)) {
                result.invert_logic_temp = ['OFF', 'ON'][msg.data[0x0225]];
            }
            return result;
        },
    },
	temperaturef_config: {
        cluster: 'msTemperatureMeasurement',
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            const result = {};
            if (msg.data.hasOwnProperty(0x0210)) {
                result.temperature_offset = parseFloat(msg.data[0x0210])/10.0;
            }
            return result;
        },
    },
};

const definition = {
        zigbeeModel: ['Smuta'],
        model: 'Smuta',
        vendor: 'Okoshko Production',
        description: 'Smuta - temperature sensor, control relay',
        fromZigbee: [fz.temperature, fzLocal.termostat_config, fzLocal.node_config,
		fzLocal.temperaturef_config],
        toZigbee: [tz.factory_reset, tzLocal.termostat_config, tzLocal.node_config,
		tzLocal.temperaturef_config],
        configure: async (device, coordinatorEndpoint, logger) => {
		const endpointOne = device.getEndpoint(1);
		await reporting.bind(endpointOne, coordinatorEndpoint, ['genPowerCfg', 'msTemperatureMeasurement']);
        },
        exposes: [e.temperature(),
		exposes.numeric('report_delay', ea.STATE_SET).withUnit('Seconds').withDescription('Adjust Report Delay. Setting the time in seconds, by default 60 seconds')
			.withValueMin(10).withValueMax(3600),
		exposes.numeric('high_temp', ea.STATE_SET).withUnit('C').withDescription('Setting High Temperature Border')
			.withValueMin(-5).withValueMax(50),
		exposes.numeric('low_temp', ea.STATE_SET).withUnit('C').withDescription('Setting Low Temperature Border')
			.withValueMin(-5).withValueMax(50)],
};

module.exports = definition;
